import random

import rospy
from std_msgs.msg import String


def create_resistance():
    resistance = random.randint(100, 1000000)
    return str(resistance)


def mock_resistance(pub):
    rospy.loginfo('Mocking resistance')
    pub.publish(create_resistance())


if __name__ == '__main__':
    rospy.init_node('mock_resistance', anonymous=True)

    resistance_publisher = rospy.Publisher('resistance', String, queue_size=10)

    mock_resistance(resistance_publisher)
