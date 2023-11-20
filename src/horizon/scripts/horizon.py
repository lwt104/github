import rospy
from std_msgs.msg import Bool

rospy.init_node("horizon_publisher", anonymous=True)

horizon_publisher = rospy.Publisher('horizon', Bool, queue_size=1)

r = rospy.Rate(5) # define rate here

while not rospy.is_shutdown():
    msg=Bool
    msg.data=True
    horizon_publisher.publish(msg)
    r.sleep()