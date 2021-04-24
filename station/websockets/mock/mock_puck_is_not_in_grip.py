import json

import rospy
from std_msgs.msg import String


def create_puck_is_not_in_grip():
    movement_vectors_string = (0, 0, 8)
    return json.dumps(movement_vectors_string)


def mock_puck_is_not_in_grip(pub):
    rospy.loginfo('Mocking movement_vectors_string (puck not in grip)')
    pub.publish(create_puck_is_not_in_grip())


if __name__ == '__main__':
    rospy.init_node('mock_puck_is_not_in_grip', anonymous=True)

    movement_publisher = rospy.Publisher('movement_vectors_string', String, queue_size=10)

    mock_puck_is_not_in_grip(movement_publisher)
