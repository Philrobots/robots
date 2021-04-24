import rospy
from std_msgs.msg import Bool


# TODO : Remove this mock
if __name__ == '__main__':
    start_publisher = rospy.Publisher('start_cycle', Bool, queue_size=10)

    rospy.init_node('mock_start', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        start_publisher.publish(True)
        rate.sleep()
