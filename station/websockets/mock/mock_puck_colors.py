import json
import random
from enum import Enum
import rospy
from std_msgs.msg import String


class Color(Enum):
    yellow = 0
    brown = 1
    red = 2
    pink = 3
    orange = 4
    black = 5
    white = 6
    green = 7
    blue = 8
    purple = 9
    grey = 10


def create_puck_colors():
    puck_colors = [
        random.choice(list(Color)).name,
        random.choice(list(Color)).name,
        random.choice(list(Color)).name
    ]
    return json.dumps(puck_colors)


def mock_puck_colors(pub):
    rospy.loginfo('Mocking puck_colors')
    pub.publish(create_puck_colors())


if __name__ == '__main__':
    rospy.init_node('mock_puck_colors', anonymous=True)

    puck_colors_publisher = rospy.Publisher('puck_colors', String, queue_size=10)

    mock_puck_colors(puck_colors_publisher)
