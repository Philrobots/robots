import random
from enum import Enum

import rospy
from std_msgs.msg import String


class Step(Enum):
    CycleNotStarted = 'CycleNotStarted'
    CycleReadyInWaitingMode = 'CycleReadyInWaitingMode'
    CycleStarted = 'CycleStarted'
    ToResistanceStation = 'ToResistanceStation'
    ReadResistance = 'ReadResistance'
    ToControlPanel = 'ToControlPanel'
    ReadControlPanel = 'ReadControlPanel'
    ToFirstPuckAndGrabFirstPuck = 'ToFirstPuckAndGrabFirstPuck'
    ToFirstCornerAndReleaseFirstPuck = 'ToFirstCornerAndReleaseFirstPuck'
    ToSecondPuckAndGrabSecondPuck = 'ToSecondPuckAndGrabSecondPuck'
    ToSecondCornerAndReleaseSecondPuck = 'ToSecondCornerAndReleaseSecondPuck'
    ToThirdPuckAndGrabThirdPuck = 'ToThirdPuckAndGrabThirdPuck'
    ToThirdCornerAndReleaseThirdPuck = 'ToThirdCornerAndReleaseThirdPuck'
    ToSquareCenter = 'ToSquareCenter'
    CycleEndedAndRedLedOn = 'CycleEndedAndRedLedOn'


def create_current_step():
    return random.choice(list(Step)).name


def mock_current_step(pub, step=create_current_step()):
    rospy.loginfo('Mocking current_step: {}'.format(step))
    pub.publish(step)


if __name__ == '__main__':
    rospy.init_node('mock_current_step', anonymous=True)

    current_step_publisher = rospy.Publisher('current_step', String, queue_size=10)

    mock_current_step(current_step_publisher)
