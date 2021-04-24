class MarkerPosition:

    def __init__(self, markers_points, rotation_vector, translation_vector):
        self.markers_points = markers_points
        self.rotation_vector = rotation_vector
        self.translation_vector = translation_vector

    def set_markers_points(self, markers_points):
        self.markers_points = markers_points

    def set_rotation_vector(self, rotation_vector):
        self.rotation_vector = rotation_vector

    def get_markers_points(self):
        return self.markers_points

    def get_rotation_vector(self):
        return self.rotation_vector

    def get_translation_vector(self):
        return self.translation_vector
