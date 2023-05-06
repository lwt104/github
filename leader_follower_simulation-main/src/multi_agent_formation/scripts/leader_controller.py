#!/usr/bin/env python3

import tf
import math
import rospy
import argparse
import numpy as np
from scipy import interpolate
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from tf2_msgs.msg import TFMessage
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, Quaternion, TransformStamped
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from tf.transformations import quaternion_from_euler, euler_from_quaternion, translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_matrix

# Constants
NUM_FOLLOWERS = 3
FORMATION = [[-1, -1], 
             [-1, 1],
             [-2, 0]]

INIT_POSITION = [-1, 1, 0]  # in world frame
_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def get_model_state(model_name: str = 'robot_1'):
  rospy.wait_for_service("/gazebo/get_model_state")
  try:
      pose = _model_state(model_name, 'world').pose
      position = [pose.position.x, pose.position.y]
      rotation = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
      return position, rotation
  except (rospy.ServiceException):
      rospy.logwarn("/gazebo/get_model_state service call failed")


def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  fig.canvas.draw()


class LeaderController:
    def __init__(self, leader_id: int = 1):
        # rospy.init_node(f'leader', anonymous=True)
        rospy.init_node(f'leader')
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(10)
        self.leader_id = leader_id
        self.robot_namespace = f'robot_{leader_id}'
        self.odom_frame = f'/{self.robot_namespace}/odom'
        self.base_frame = f'/{self.robot_namespace}/base_link'          
        self.goal = None
        self.t = None
        self.trajectory = None     

        # Leader gain
        self.Kl = 0.1

        # Trajectory interpolation parameters
        self.time_ahead = 0.3
        
        # PID control parameters
        self.Kpv = 1.0
        self.Kiv = 0.0
        self.Kdv = 0.0
        self.Kpw = 0.5
        self.Kiw = 0.0
        self.Kdw = 0.0    
        self.Kpy = 0
        self.Kpa = 0.1
        
        self.integral = 0.0
        self.previous_error = 0.0

        # Min and max linear and angle velocity
        self.min_v = 0.0
        self.max_v = 1.2
        self.max_w = 1.0
        
        rospy.Subscriber(f'/{self.robot_namespace}/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber(f'/{self.robot_namespace}/move_base/TebLocalPlannerROS/teb_feedback', FeedbackMsg, self.traj_callback)
        self.tf_listener = tf.TransformListener()
        self.traj_pubs = []
        self.path_pubs = []
        for i in range(NUM_FOLLOWERS):
            self.traj_pubs.append(rospy.Publisher(f'/robot_{i+2}/trajectory', TrajectoryMsg, queue_size=10))
            self.path_pubs.append(rospy.Publisher(f'/robot_{i+2}/visual_path', Path, queue_size=10))
        self.path_pub = rospy.Publisher(f'/{self.robot_namespace}/visual_path', Path, queue_size=10)
        self.stop_pub = rospy.Publisher(f'/stop', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(f'/{self.robot_namespace}/cmd_vel', Twist, queue_size=10)
        self.run()
        rospy.spin()
        
    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Shutting down leader node!")
        rospy.sleep(1)

    def goal_callback(self, data):
        self.goal = data.pose
        print('Goal received!')
        print(self.goal)

    def traj_callback(self, data):
        print('Trajectory received!')   
        if not data.trajectories: # empty
            self.trajectory = []
            self.stamp = rospy.Time(0)
        else:
            self.trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
            self.stamp = data.trajectories[data.selected_trajectory_idx].header.stamp
        # path = self.traj_to_path(data.trajectories[data.selected_trajectory_idx])
        # self.path_pub.publish(path)
        self.send_traj()    
        time_start = rospy.Time.now().to_sec()
        self.t, self.x, self.y, self.theta, self.vel, self.omega = self.interpolate_traj(self.trajectory)  
        time_end = rospy.Time.now().to_sec()
        time_delay = time_end - self.stamp.to_sec()
        print('Interpolation time: ', time_end - time_start)
        print('Time delay', time_delay)

    def traj_to_path(self, traj):
        # Visualize the trajectory as a path to debug
        path = Path()
        path.header = traj.header
        for point in traj.trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = point.pose.position
            pose_stamped.pose.orientation = point.pose.orientation
            path.poses.append(pose_stamped)
        return path 

    def interpolate_traj(self, traj):
        # Get the traj info (time, position, velocity, omega)
        traj_t = []
        traj_x = []
        traj_y = []
        traj_theta = []
        traj_vel = []
        traj_omega = []
        for point in traj:
            traj_t.append(point.time_from_start.to_sec())
            traj_x.append(point.pose.position.x)
            traj_y.append(point.pose.position.y)
            traj_theta.append(euler_from_quaternion([point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z, point.pose.orientation.w])[2])
            traj_vel.append(point.velocity.linear.x)
            traj_omega.append(point.velocity.angular.z)

        # # Use arctan2 to avoid discontinuity
        # traj_theta = np.arctan2(np.gradient(traj_y), np.gradient(traj_x))    

        try:
            # Interpolate the traj
            x = interpolate.splrep(traj_t, traj_x)
            y = interpolate.splrep(traj_t, traj_y)
            theta = interpolate.splrep(traj_t, traj_theta)
            vel = interpolate.splrep(traj_t, traj_vel)
            omega = interpolate.splrep(traj_t, traj_omega)
        except:
            x = traj_x
            y = traj_y
            theta = traj_theta
            vel = traj_vel
            omega = traj_omega
        return traj_t, x, y, theta, vel, omega

    def send_traj(self): 
        for i in range(NUM_FOLLOWERS):
            traj = TrajectoryMsg()
            traj.header.stamp = self.stamp
            traj.header.frame_id = f'robot_{i+2}/odom'
            # Transform leader trajectory to follower trajectory
            for point in self.trajectory:
                point_leader = PoseStamped()
                point_leader.header.frame_id = self.odom_frame
                point_leader.header.stamp = rospy.Time()
                point_leader.pose.position.x = point.pose.position.x + FORMATION[i][0]
                point_leader.pose.position.y = point.pose.position.y + FORMATION[i][1]
                point_leader.pose.position.z = point.pose.position.z
                point_leader.pose.orientation = point.pose.orientation
                point_follower = TrajectoryPointMsg()
                try:
                    # print('begin to transform')
                    self.tf_listener.waitForTransform(self.odom_frame, f'/robot_{i+2}/odom', rospy.Time(), rospy.Duration(0.3))
                    pose = self.tf_listener.transformPose(f'robot_{i+2}/odom', point_leader)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("TF Exception")
                    continue                
                point_follower.pose.position = pose.pose.position
                point_follower.pose.orientation = pose.pose.orientation
                point_follower.velocity = point.velocity
                point_follower.acceleration = point.acceleration
                point_follower.time_from_start = point.time_from_start
                traj.trajectory.append(point_follower)
            
            time_delay = rospy.get_rostime().to_sec() - self.stamp.to_sec()
            # print('time_delay', time_delay)
            path = self.traj_to_path(traj)
            self.traj_pubs[i].publish(traj)
            self.path_pubs[i].publish(path)
            
    def get_odom(self):
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(0.3))
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
            return np.array(trans[:2]), rotation[2]
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return 
        
    def angle_clip(self, angle):
        if angle > 0:
            if angle > np.pi:
                return angle - 2*np.pi
        else:
            if angle < -np.pi:
                return angle + 2*np.pi
        return angle

    def stop(self, position, rotation):
        # Publish stop flag
        dx = self.goal.position.x - position[0]
        dy = self.goal.position.y - position[1]
        dtheta = euler_from_quaternion([self.goal.orientation.x, self.goal.orientation.y, self.goal.orientation.z, self.goal.orientation.w])[2] - rotation
        dtheta = self.angle_clip(dtheta)
        dist = np.sqrt(dx**2 + dy**2) 
        if dist < 0.2 and np.abs(dtheta) < 0.3:
            rospy.loginfo("Goal reached!")
            stop_flag = True
        else:
            stop_flag = False
        return stop_flag  

    def run(self):
        while not rospy.is_shutdown():
            if self.goal and self.t:
                print('-------- begin to track the waypoint --------')
                
                vel_msg = Twist()
                # Get the pose of the robot
                # position, rotation = self.get_odom()
                position, rotation = get_model_state()
                # x0 = position[0] - INIT_POSITION[0] # Both coordinate take the initial point as the origin
                # y0 = position[1] - INIT_POSITION[1] # Right is positive x, forward is positive y      
                x0 = position[0] 
                y0 = position[1]       
                # Get the current time
                current_time = rospy.Time.now()
                # Get the time difference
                dt = (current_time - self.stamp).to_sec()
                # print('time delay', dt)
                # Judge whether to stop
                stop_flag = self.stop(position, rotation)
                if dt > self.t[-1]:
                    rospy.loginfo("Time out, stop the robot!")
                    stop_flag = True
                stop = Bool()
                stop.data = stop_flag
                self.stop_pub.publish(stop)  

                if not stop_flag:
                    # Get the time ahead
                    t_ahead = dt + self.time_ahead
                    print('time ahead', t_ahead)
                    # Calculate the desired linear and angular velocities using PID control
                    try:
                        x_t = interpolate.splev(t_ahead, self.x)
                        y_t = interpolate.splev(t_ahead, self.y)
                        theta_t = interpolate.splev(t_ahead, self.theta)
                        vel_t = interpolate.splev(t_ahead, self.vel)
                        omega_t = interpolate.splev(t_ahead, self.omega)
                    except:
                        if self.t[-1] <= t_ahead:
                            idx = -1
                        else:
                            idx = np.where(np.array(self.t) > t_ahead)[0][0]
                        x_t = self.x[idx]
                        y_t = self.y[idx]
                        theta_t = self.theta[idx]
                        vel_t = self.vel[idx]
                        omega_t = self.omega[idx]

                    # Calculate the desired linear and angular velocities using PID control
                    dx = x_t - x0
                    dy = y_t - y0
                    dist = np.sqrt(dx**2 + dy**2)
                    fai = np.arctan2(dy, dx)
                  
                    alpha = self.angle_clip(fai - rotation)
                    x_error = dist * np.cos(alpha)
                    # y_error = dist * np.sin(alpha)
                    #x_error = dx
                    y_error = dy
                    ang_error = self.angle_clip(theta_t - rotation)
                    self.integral += x_error * self.time_ahead
                    derivative = (x_error - self.previous_error) / self.time_ahead
                    linear_velocity = self.Kl * vel_t + self.Kpv * x_error + self.Kiv * self.integral + self.Kdv * derivative
                    # angular_velocity = self.Kl * omega_t + self.Kpw * ang_error + self.Kpy * y_error
                    angular_velocity = self.Kl * omega_t + self.Kpw * alpha + self.Kpy * fai + self.Kpa * ang_error
                    # print('integral', self.integral)
                    # print('derivative', derivative)
                    print('error', x_error, y_error, ang_error)
                    print('vel', vel_t, linear_velocity)
                    print('ang_vel', omega_t, angular_velocity, alpha, fai, rotation)
                    print('dx', dx, dy)
                                                
                    # Publish the linear and angular velocities to the follower's velocity topic
                    vel_msg.linear.x = np.clip(linear_velocity, -self.max_v, self.max_v)
                    vel_msg.angular.z = np.clip(angular_velocity, -self.max_w, self.max_w) 
                        

                    self.previous_error = x_error
                
                self.cmd_vel_pub.publish(vel_msg)     

            self.rate.sleep()   

if __name__=="__main__":
    try:
        leader = LeaderController()
    except rospy.ROSInterruptException:
        pass