import json
import random
import rospy
from std_msgs.msg import String


def create_robot_consumption():
    return json.dumps(
        {
            "robotConsumption": {
                "wheel1": random.uniform(0, 100),
                "wheel2": random.uniform(0, 100),
                "wheel3": random.uniform(0, 100),
                "wheel4": random.uniform(0, 100),
                "total": random.uniform(0, 400),
                "remainingTime": random.randint(0, 3600),
                "batteryCharge": random.randint(0, 8)
            }
        }
    )


# TODO : Remove this mock
if __name__ == '__main__':
    ready_publisher = rospy.Publisher('robot_consumption', String, queue_size=10)

    rospy.init_node('mock_robot_consumption', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        ready_publisher.publish(create_robot_consumption())
        rate.sleep()
