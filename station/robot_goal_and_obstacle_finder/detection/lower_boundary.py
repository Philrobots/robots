import numpy as np


class LowerBoundary:
    """Lower boundaries of colors of pucks"""
    def __init__(self):
        self.lower_boundaries = {"purple": [117, 20, 94], "white": [20, 0, 155],
                                 "yellow": [19, 105, 40],
                                 "blue": [101, 50, 0], "orange": [0, 82, 110],
                                 "red": [0, 216, 92],
                                 "brown": [0, 121, 15], "green": [46, 65, 5],
                                 "black": [31, 21, 0],
                                 "grey": [11, 18, 27],
                                 "square": [57, 33, 13],
                                 "table": [7, 0, 95]}

    def get_lower_boundaries(self, color_to_detect):
        if color_to_detect in self.lower_boundaries:
            return np.array(self.lower_boundaries[color_to_detect])
        return np.zeros(3)
