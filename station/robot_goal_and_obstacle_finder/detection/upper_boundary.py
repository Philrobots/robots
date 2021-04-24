import numpy as np


class UpperBoundary:
    """Upper boundaries of colors of pucks"""
    def __init__(self):
        self.upper_boundaries = {"purple": [143, 255, 233], "white": [124, 62, 186],
                                 "yellow": [41, 255, 255],
                                 "blue": [179, 255, 255], "orange": [19, 255, 153],
                                 "red": [179, 255, 106],
                                 "brown": [26, 255, 39], "green": [75, 255, 255],
                                 "black": [126, 255, 17],
                                 "grey": [56, 108, 96],
                                 "square": [104, 255, 125],
                                 "table": [69, 79, 232]}

    def get_upper_boundaries(self, color_to_detect):
        if color_to_detect in self.upper_boundaries:
            return np.array(self.upper_boundaries[color_to_detect])
        return np.zeros(3)
