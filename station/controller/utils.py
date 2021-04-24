from geometry_msgs.msg import PoseStamped


def create_pose(position):
    pose = PoseStamped()
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    return pose
