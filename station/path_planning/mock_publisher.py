import json
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from std_msgs.msg import String


def create_robot_pose():
    pose = PoseStamped()
    pose.pose.position.x = 20
    pose.pose.position.y = 240
    pose.header = "map"
    return pose


def create_obstacles_poses():
    pose_array = PoseArray()

    pose1 = Pose()
    pose1.position.x = 320
    pose1.position.y = 240

    pose2 = Pose()
    pose2.position.x = 200
    pose2.position.y = 480

    pose_array.poses = [pose1, pose2]
    return pose_array


def create_pucks_poses():
    pose_array = PoseArray()

    pose1 = Pose()
    pose1.position.x = 630
    pose1.position.y = 20

    pose2 = Pose()
    pose2.position.x = 630
    pose2.position.y = 460

    pose_array.poses = [pose1, pose2]
    return pose_array


def create_goal_pose():
    pose = PoseStamped()
    pose.pose.position.x = 1250
    pose.pose.position.y = 400
    return pose

def create_pose(position):
    pose = PoseStamped()
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    return pose



class Publisher:
    def __init__(self):
        self.puck = None
        rospy.Subscriber('pucks', String, self.callback_pucks)
        self.sauce = True
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    def callback_pucks(self, pucks):
        if self.sauce:
            pucks_dict = json.loads(str(pucks.data))
            self.puck = pucks_dict["black"][0]["center_position"]
            print(self.puck)
            self.goal_publisher.publish(create_pose(self.puck))
            print("sent goal")
            self.sauce = False


# TODO : Remove this mock (and usage)
if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    Publisher()
    rospy.spin()
