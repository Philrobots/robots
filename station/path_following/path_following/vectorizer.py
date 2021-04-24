import math

from path_following.config import NODE_SIZE
from path_following.destination import Destination
from path_following.movement_mode import MovementMode


class Vectorizer:
    def __init__(self, minimize=False):
        self.robot_position = None
        self.robot_angle = None
        self.minimize = minimize
        self.path = []
        self.mode = MovementMode.GRIP
        self.cm_to_pixel = 6.882391855
        self.distance_correction_threshold = 3*self.cm_to_pixel
        self.length_correction_threshold = 0
        self.angle_correction_threshold = 0
        self.checkpoint_trigger_threshold = 55
        self.last_checkpoint = None
        self.goal = None
        self.checkpoint = None

        self.destination = Destination.OTHER

        self.objective = None

    def set_mode(self, mode: MovementMode):
        self.mode = mode

    def set_destination_mode(self, destination: Destination):
        self.destination = destination

    def set_robot_position(self, position: (int, int)):
        self.robot_position = position

        # update checkpoint
        node_distances = [
            i for i, node in enumerate(self.path) if distance(self.robot_position, node) <= self.checkpoint_trigger_threshold
        ]
        # node_distances = [
        #     i for i, node in enumerate(self.path) if self.robot_position[0] >= node[0] or distance(self.robot_position, node) <= 50
        # ]
        if node_distances:
            if self.checkpoint is None or node_distances[-1] > self.checkpoint:
                self.checkpoint = node_distances[-1]

    def set_path(self, path: [(float, float)]):
        # if self.destination is Destination.PUCK:
        path = self.shorten_path_to_grab_puck(path)
        self.path = self.minimize_path(path)
        # # if self.destination is Destination.PUCK or self.destination is Destination.CORNER:
        # # else:
        # #     self.path = path
        self.objective = None
        self.checkpoint = None

    def minimize_path(self, path: [[float, float]]):
        if path and len(path) > 1:
            minimized_path = [path[0]]
        elif len(path) == 1:
            path = [self.robot_position, path[0]]
        current_angle = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
        for i, node in enumerate(path[2:-1]):
            x, y = node

            x2, y2 = path[i+1]
            angle_between_last_one = math.atan2(y-y2, x-x2)

            if angle_between_last_one == current_angle:
                current_angle = angle_between_last_one
                continue
            current_angle = angle_between_last_one
            minimized_path.append((x2, y2))

        if path[-1] != minimized_path[-1]:
            minimized_path.append(path[-1])
        return minimized_path

    def shorten_path_to_grab_puck(self, path: [(float, float)]):
        distances = [distance(node, self.goal) for node in path]
        new_path = []
        for i, distance_from_goal in enumerate(distances):
            new_path.append(path[i])
            if distance_from_goal <= 112:
                break
        return new_path

    def set_goal(self, goal: (int, int)):
        self.goal = goal

    def set_destination(self, destination: Destination):
        self.destination = destination

    def minimize_vectors(self, vectors: [[float, float]]):
        minimized_vectors = []
        for i, vector in enumerate(vectors):
            length, angle = vector
            if angle != 0:
                minimized_vectors.append(vector)
            elif angle == 0:
                if not minimized_vectors:
                    minimized_vectors.append(vector)
                else:
                    last_vector_distance, last_vector_angle = minimized_vectors[-1]
                    minimized_vectors[-1] = [last_vector_distance + length, last_vector_angle]
        return minimized_vectors

    def get_path_from_robot(self, nodes: [(float, float)]):
        if self.checkpoint is None:
            self.checkpoint = 0

        node_distances = [
            distance(self.robot_position, node)
            for node in nodes[self.checkpoint+1:]
        ]

        if not node_distances:
            return [self.robot_position]

        minimum_distance = min(node_distances)
        index = node_distances.index(minimum_distance) + self.checkpoint + 1

        self.objective = nodes[index]
        if self.robot_is_close_to_path(minimum_distance):
            if self.checkpoint_was_updated_recently():
                return nodes[self.checkpoint+1:]
            else:
                return nodes[index:]
        else:
            return nodes[index:]

    def checkpoint_was_updated_recently(self):
        return distance(self.robot_position, self.path[self.checkpoint]) <= NODE_SIZE

    def robot_is_close_to_path(self, minimum_distance_from_path):
        return minimum_distance_from_path <= self.distance_correction_threshold

    def vectorize(self, nodes: [(float, float)]):
        vectors = []
        for i in range(len(nodes)-1):
            x1, y1 = nodes[i]
            x2, y2 = nodes[i+1]
            length = ((x2-x1)**2 + (y2-y1)**2)**0.5
            angle = -math.atan2(y2-y1, x2-x1)

            if angle == -0:
                angle = 0
            elif angle == -math.pi:
                angle = math.pi

            vector = [length, angle]
            vectors.append(vector)
        return vectors

    def adjust_vector_angles_from_robot_pov(self, vectors: [[float, float]]):
        """
        Changes the vectors' orientation from their absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For all vectors)
        """
        new_vectors = []
        if self.mode is MovementMode.OHMMETER:
            last_vector = [None, self.robot_angle - math.pi/2]
        else:
            last_vector = [None, self.robot_angle]

        for vector in vectors:
            length, angle = self.adjust_vector_angle_from_robot_pov(last_vector, vector)
            if abs(angle) <= self.angle_correction_threshold:
                angle = 0
            if abs(length) <= self.length_correction_threshold:
                length = 0
            new_vectors.append([length, angle, self.mode])
            last_vector = vector
        return new_vectors

    def adjust_vector_angle_from_robot_pov(self, robot_angle, vector_angle):
        """
        Changes the vector orientation from the absolute value from the top camera
        to the angle the robot will need to use to align itself with the vector.
        (For one vector)
        """
        if vector_angle< 0:
            vector_angle = 2 * math.pi +vector_angle
        if robot_angle< 0:
            robot_angle = 2 * math.pi + robot_angle

        angle_correction = vector_angle - robot_angle

        if angle_correction > math.pi:
            angle_correction -= 2 * math.pi
        elif angle_correction < -math.pi:
            angle_correction += 2 * math.pi
        return angle_correction

    def set_robot_angle(self, robot_angle):
        self.robot_angle = robot_angle

    def path_to_vectors(self):
        path_from_robot = self.get_path_from_robot(self.path)

        if self.destination is Destination.PUCK or self.destination is Destination.CORNER:
            tuple_length_angle = self.calculate_distance_and_angle()
            return [(tuple_length_angle[0], tuple_length_angle[1], MovementMode.GRIP)]

        elif self.destination is Destination.RESISTANCE_STATION:
            tuple_length_angle = self.calculate_distance_and_angle()
            return [(tuple_length_angle[0], tuple_length_angle[1], MovementMode.GRIP)]
        else:
            tuple_length_angle = self.calculate_distance_and_angle()
            return [(tuple_length_angle[0], tuple_length_angle[1], MovementMode.GRIP)]

    def calculate_distance_and_angle(self):
        if self.objective is not None:
            xp, yp = self.robot_position
            xg, yg = self.objective

            length = ((xg-xp)**2 + (yg-yp)**2)**0.5

            angle_correction = self.find_goal_angle(yg - yp, xg- xp, self.destination)

            return length, angle_correction
        else:
            raise Exception("wtf, objective is none")

    def find_goal_angle(self, diff_y, diff_x, destination):
        angle = -math.atan2(diff_y, diff_x)
        if angle == -0:
            angle = 0
        elif angle == -math.pi:
            angle = math.pi

        if angle < 0:
            angle = 2 * math.pi + angle
        if self.robot_angle < 0:
            self.robot_angle = 2 * math.pi + self.robot_angle

        angle_correction = angle - self.robot_angle

        if angle_correction > math.pi:
            angle_correction -= 2 * math.pi
        elif angle_correction < -math.pi:
            angle_correction += 2 * math.pi

        if destination == "RESISTANCE":
            angle_correction -= math.pi/2
        elif destination == "CENTER":
            pass
        else:
            pass

        return angle_correction

    def robot_is_on_goal(self):
        """
        # TODO:
        We don't do a final angle correction if robot_is_on_goal because it will always want to correct the
        angle to 0 degrees. We can actually try to toggle this on/off in the lab to see the robot's
        actual behavior.
        """
        return distance(self.robot_position, self.goal) <= NODE_SIZE/2


def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
