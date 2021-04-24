import json
import random
from enum import Enum
import rospy
from std_msgs.msg import String


class Corner(Enum):
    A = 'A'
    B = 'B'
    C = 'C'
    D = 'D'


def create_letters():
    letters = [
        random.choice(list(Corner)).name,
        random.choice(list(Corner)).name,
        random.choice(list(Corner)).name
    ]
    return json.dumps(letters)


def mock_letters(pub):
    rospy.loginfo('Mocking letters')
    pub.publish(create_letters())


if __name__ == '__main__':
    rospy.init_node('mock_letters', anonymous=True)

    letters_publisher = rospy.Publisher('letters', String, queue_size=10)

    mock_letters(letters_publisher)
