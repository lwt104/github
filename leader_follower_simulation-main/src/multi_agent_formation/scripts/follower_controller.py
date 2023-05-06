#!/usr/bin/env python3

import tf
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

INIT_POSITION = [0, 0, 0]  # in world frame
_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# def get_model_state(model_name: str = 'robot_2'):
#   rospy.wait_for_service("/gazebo/get_model_state")
#   try:
#       pose = _model_state(model_name, 'world').pose
#       position = [pose.position.x, pose.position.y]
#       rotation = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
#       return position, rotation
#   except (rospy.ServiceException):
#       rospy.logwarn("/gazebo/get_model_state service call failed")
  
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


class FollowerController:
    def __init__(self, follower_id, leader_id: int = 1):
        # rospy.init_node(f'follower_{follower_id}', anonymous=True)
        rospy.init_node(f'follower_{follower_id}')
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(50)       
        self.follower_id = follower_id
        self.robot_namespace = f'robot_{follower_id}'
        self.leader_id = leader_id
        self.odom_frame = f'/{self.robot_namespace}/odom'
        self.base_frame = f'/{self.robot_namespace}/base_link'
        self.t = None
        self.trajectory = None
        self.stop_flag = False
        
        # Leader gain
        self.Kl = 0.1
        self.Klw = 0.5

        # Trajectory interpolation parameters
        self.time_ahead = 0.25        
        
        # PID control parameters
        self.Kpv = 1.0
        self.Kiv = 0.0
        self.Kdv = 0.0
        self.Kpw = 0.1
        self.Kiw = 0.0
        self.Kdw = 0.0
        self.Kpy = 0.1 
        self.Kpa = 0.1 
        self.integral = 0.0
        self.previous_error = 0.0

        # Artificial Potential Field parameters
        self.att = 0.5
        self.rep = 3.0
        self.footprint = [[-0.21, -0.165], [-0.21, 0.165], [0.21, 0.165], [0.21, -0.165]]
        self.min_lidar_range = 0.05
        self.max_lidar_range = 3.0
        self.robot_R = 0.4 # 0.389
        self.step_size = 0.1
        self.obstacle_safe_R = 0.6
        self.obstacle_detect_R = 0.8

        # Min and max linear and angle velocity
        self.min_v = 0.1
        self.max_v = 1.3
        self.max_w = 1.0

        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(f'/{self.robot_namespace}/trajectory', TrajectoryMsg, self.traj_callback)
        rospy.Subscriber(f'/{self.robot_namespace}/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/stop', Bool, self.stop_callback)
        self.cmd_vel_pub = rospy.Publisher(f'/{self.robot_namespace}/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher(f'/{self.robot_namespace}/visual_path', Path, queue_size=10)
        #self.obstacle_pub = rospy.Publisher(f'/{self.robot_namespace}/obstacles', PointCloud, queue_size=10)
        #self.waypoint_pub = rospy.Publisher(f'/{self.robot_namespace}/waypoint', PointStamped, queue_size=10)
        #self.footprint_pub = rospy.Publisher(f'/{self.robot_namespace}/footprint', PolygonStamped, queue_size=10)
        self.run()
        rospy.spin()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Shutting down follower node!")
        rospy.sleep(1) 
         
    def stop_callback(self, data):
        self.stop_flag = data.data
        if self.stop_flag:
            rospy.loginfo("Stop signal received!")
            self.cmd_vel_pub.publish(Twist())
    
    def scan_callback(self, data):
        self.footprint_odom, self.obstacles = self.detect_obstacles(data)

    def traj_callback(self, data):
        print('Trajectory received!')
        if not data.trajectory: # empty
            self.trajectory = []
            self.stamp = rospy.Time(0)
        else:
            self.trajectory = data.trajectory
            self.stamp = data.header.stamp
        # path = self.traj_to_path(data)
        # self.path_pub.publish(path)     
        time_start = rospy.Time.now().to_sec()
        self.t, self.x, self.y, self.theta, self.vel, self.omega = self.interpolate_traj(self.trajectory)  
        time_end = rospy.Time.now().to_sec()
        time_delay = time_end - self.stamp.to_sec()
        print('Interpolation time: ', time_end - time_start)
        print('Time delay', time_delay)

    def traj_to_path(self, traj):
        # Visualize the trajectory as a path
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
        # Consider obstacle avoidance when interpolating the trajectory
        # _, self.obstacles = self.detect_obstacles()        
        for point in traj:
            pos = [point.pose.position.x, point.pose.position.y]
            ang = euler_from_quaternion([point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z, point.pose.orientation.w])[2]
            flag = True
            while flag:
                footprint = np.array(pos) + np.dot(self.footprint, self.rotation_matrix(-ang))
                center = np.mean(footprint, axis=0)
                dists = [np.sqrt((center[0] - obs[0])**2 + (center[1] - obs[1])**2) for obs in self.obstacles]
                min_dist = min(dists)
                # If the waypoint is too close to an obstacle, change it to a safe position
                if min_dist < self.obstacle_safe_R:
                    closest_obstacle = self.obstacles[np.argmin(dists)]
                    # repulsive_force = self.calculate_repulsive_force(pos, self.obstacles)
                    # direction = repulsive_force / np.linalg.norm(repulsive_force)
                    direction = (pos - closest_obstacle) / min_dist
                    pos = pos + direction * self.step_size
                    rep_ang1 = np.arctan2(direction[1], direction[0]) + np.pi/2
                    rep_ang2 = np.arctan2(direction[1], direction[0]) - np.pi/2
                    diff1 = self.angle_clip(rep_ang1 - ang)
                    diff2 = self.angle_clip(rep_ang2 - ang)
                    if np.abs(diff1) < np.abs(diff2):
                        ang = self.angle_clip(rep_ang1)
                    else:
                        ang = self.angle_clip(rep_ang2)
                    # ang = 0.3*rep_ang + 0.7*self.angle_clip(ang)
                else:
                    flag = False
            # ! Visualize footprint to debug        
            self.visualize_footprint(footprint)
                    
            traj_t.append(point.time_from_start.to_sec())
            traj_x.append(pos[0])
            traj_y.append(pos[1])
            traj_theta.append(ang)
            traj_vel.append(point.velocity.linear.x)
            traj_omega.append(point.velocity.angular.z)

        # # Use arctan2 to avoid discontinuity
        # traj_theta = np.arctan2(np.gradient(traj_y), np.gradient(traj_x))
        # ! Visualize path to debug
        self.visualize_path(traj_x, traj_y)
        
        try:
            # Interpolate the trajectory
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

    def get_odom(self):
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(0.3))
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
            return np.array(trans[:2]), rotation[2]
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

    def rotation_matrix(self, angle):
        c = np.cos(angle)
        s = np.sin(angle)
        return np.array([[c, -s], [s, c]])

    def angle_clip(self, angle):
        if angle > 0:
            if angle > np.pi:
                return angle - 2*np.pi
        else:
            if angle < -np.pi:
                return angle + 2*np.pi
        return angle

    def detect_obstacles(self, scan):
        # scan = rospy.wait_for_message('/scan', LaserScan)
        # Convert laser scan data to Cartesian coordinates in robot frame
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        effective_idxs = np.where((ranges > self.min_lidar_range) & (ranges < self.max_lidar_range))[0]
        ranges = ranges[effective_idxs]
        angles = angles[effective_idxs]
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        # Calculate obstacles in robot frame
        obstacles = np.array([x, y]).T  
        # Convert footprint and obstacles to odom frame
        position, rotation = self.get_odom()
        footprint = np.array(position) + np.dot(self.footprint, self.rotation_matrix(-rotation))
        obstacles = np.array(position) + np.dot(obstacles, self.rotation_matrix(-rotation))
        # ! Visualize obstacles to check if they are in odom frame
        # self.visualize_obstacles(obstacles)
        return footprint, obstacles

    def calculate_repulsive_force(self, position, obstacles):
        # Calculate repulsive force from obstacles  
        repulsive_force = np.zeros(2)
        for obstacle in obstacles:
            dist = np.linalg.norm(position - obstacle)
            if dist < self.obstacle_detect_R:
                # force =  self.rep * (1 - dist/self.obstacle_detect_R)**2
                force = self.rep * (1 / dist - 1 / self.obstacle_detect_R) / dist**2
                direction = (position - obstacle) / dist
                repulsive_force += force * direction          
        return repulsive_force

    def calculate_attractive_force(self, position, goal):
        # Calculate attractive force from goal  
        attractive_force = self.att * (goal - position)       
        return attractive_force

    def visualize_path(self, x, y):
        path_follower = Path()
        path_follower.header.stamp = self.stamp
        path_follower.header.frame_id = self.odom_frame
        for i in range(len(x)):
            point = PoseStamped()
            point.pose.position.x = x[i]
            point.pose.position.y = y[i]
            path_follower.poses.append(point)
        self.path_pub.publish(path_follower)  

    # def visualize_obstacles(self, obstacles):
    #     # Publish obstacles as a point cloud
    #     points = []
    #     for obstacle in obstacles:
    #         point = Point32()
    #         point.x = obstacle[0]
    #         point.y = obstacle[1]
    #         point.z = 0.0
    #         points.append(point)
    #     point_cloud = PointCloud()
    #     point_cloud.header.frame_id = self.odom_frame
    #     point_cloud.points = points
    #     self.obstacle_pub.publish(point_cloud)

    # def visualize_waypoint(self, waypoint):
    #     # Publish waypoint
    #     point = PointStamped()
    #     point.header.frame_id = self.odom_frame
    #     point.point.x = waypoint.pose.position.x
    #     point.point.y = waypoint.pose.position.y
    #     point.point.z = 0.0
    #     self.waypoint_pub.publish(point)

    # def visualize_footprint(self, footprint):
    #     polygon = PolygonStamped()
    #     polygon.header.frame_id = self.odom_frame
    #     for pos in footprint:
    #         point = Point32()
    #         point.x = pos[0]
    #         point.y = pos[1]
    #         polygon.polygon.points.append(point)
    #     self.footprint_pub.publish(polygon)

    def run(self):
        while not rospy.is_shutdown():
            if self.t:
                vel_msg = Twist()
                # Get the pose of the robot
                position, rotation = self.get_odom()                
                # # Avoid collision
                # # self.footprint_odom, self.obstacles = self.detect_obstacles()
                # center = np.mean(self.footprint_odom, axis=0)
                # dists = [np.sqrt((center[0] - obs[0])**2 + (center[1] - obs[1])**2) for obs in self.obstacles]
                # min_dist = min(dists) if len(dists) > 0 else 100
                # # If the waypoint is too close to an obstacle, change it to a safe position
                # if min_dist < self.obstacle_safe_R:
                #     print('-------- begin to avoid collision --------')
                #     closest_obstacle = self.obstacles[np.argmin(dists)]
                #     # repulsive_force = self.calculate_repulsive_force(pos, self.obstacles)
                #     # direction = repulsive_force / np.linalg.norm(repulsive_force)
                #     direction = (position - closest_obstacle) / min_dist
                #     rep_ang1 = np.arctan2(direction[1], direction[0]) + np.pi/2
                #     rep_ang2 = np.arctan2(direction[1], direction[0]) - np.pi/2
                #     diff1 = self.angle_clip(rep_ang1 - rotation)
                #     diff2 = self.angle_clip(rep_ang2 - rotation)
                #     if np.abs(diff1) < np.abs(diff2):
                #         ang = diff1
                #     else:
                #         ang = diff2                                     
                #     vel_msg.linear.x = 0.0
                #     vel_msg.angular.z = 0.2 * ang
                # else:    

                print('-------- begin to track the waypoint --------')
                # self.t, self.x, self.y, self.theta, self.vel, self.omega = self.interpolate_traj(self.trajectory)
                
                # Get the current time
                current_time = rospy.Time.now()
                # print('debug', current_time, self.stamp)
                # Get the time difference
                dt = (current_time - self.stamp).to_sec()
                print('time delay', dt)
                # Judge whether to stop
                # if dt > self.t[-1]:
                #     rospy.loginfo("Time out, stop the robot!")
                #     self.stop_flag = True
                    
                if not self.stop_flag:                    
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
                    dx = x_t - position[0]
                    dy = y_t - position[1]
                    dist = np.sqrt(dx**2 + dy**2)
                    fai = np.arctan2(dy, dx)
                    alpha = self.angle_clip(fai - rotation)
                    x_error = dist * np.cos(alpha)
                    # y_error = dist * np.sin(alpha)
                    y_error = dy
                    ang_error = self.angle_clip(theta_t - rotation)
                    self.integral += x_error * self.time_ahead
                    derivative = (x_error - self.previous_error) / self.time_ahead
                    linear_velocity = self.Kl * vel_t + self.Kpv * x_error + self.Kiv * self.integral + self.Kdv * derivative
                    # angular_velocity = self.Kl * omega_t + self.Kpw * ang_error + self.Kpy * y_error
                    #angular_velocity = self.Kl * omega_t + self.Kpw * alpha + self.Kpy * fai
                    angular_velocity = self.Klw * omega_t + self.Kpw * alpha + self.Kpy * fai + self.Kpa * ang_error
                    # print('integral', self.integral)
                    # print('derivative', derivative)
                    print('error', x_error, y_error, ang_error)
                    print('vel', vel_t, linear_velocity)
                    print('ang_vel', omega_t, angular_velocity)
                    # Publish the linear and angular velocities to the follower's velocity topic
                    vel_msg.linear.x = np.clip(linear_velocity, -self.max_v, self.max_v)
                    vel_msg.angular.z = np.clip(angular_velocity, -self.max_w, self.max_w)   

                    self.previous_error = x_error

                self.cmd_vel_pub.publish(vel_msg)     

            self.rate.sleep() 

if __name__=="__main__":
    # parser = argparse.ArgumentParser(description='Follower')
    # parser.add_argument('--follower_id', type=int, default=2, help='follower id')
    # args = parser.parse_args()    
    follower_id = rospy.get_param("~follower_id", 2)
    print(follower_id)
    try:
        follower = FollowerController(follower_id)
    except rospy.ROSInterruptException:
        pass