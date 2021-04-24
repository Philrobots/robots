class ArucoPosition:

    def __init__(self, marker_id, marker_corner):
        self.marker_id = marker_id
        self.marker_corner = marker_corner

    def get_id(self):
        return self.marker_id

    def get_corner(self):
        return self.marker_corner
