import json
import random
import rospy
from std_msgs.msg import String


def create_robot_consumption():
    robot_consumption = {
        "wheel1": random.randint(0, 10),
        "wheel2": random.randint(0, 10),
        "wheel3": random.randint(0, 10),
        "wheel4": random.randint(0, 10),
        "total": random.randint(0, 40),
        "remainingTime": random.randint(0, 600),
        "batteryCharge": random.uniform(0, 8)
    }
    return json.dumps(robot_consumption)


def mock_robot_consumption(pub):
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo('Mocking robot_consumption')
        pub.publish(create_robot_consumption())
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mock_robot_consumption', anonymous=True)

    robot_consumption_publisher = rospy.Publisher('robot_consumption', String, queue_size=10)

    mock_robot_consumption(robot_consumption_publisher)
