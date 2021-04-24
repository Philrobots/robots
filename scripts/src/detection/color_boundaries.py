class ColorBoundaries:
    """Color boundaries of pucks"""
    def __init__(self):
        self.color_boundaries = {"purple": {"lower": [110, 16, 80], "upper":[172, 245, 255]},
                                 "white": {"lower": [12, 0, 135], "upper":[96, 159, 232]} ,
                                 "yellow": {"lower": [19, 148, 60], "upper":[40, 255, 255]},
                                 "blue": {"lower": [95, 66, 36], "upper":[124, 255, 255]} ,
                                 "orange": {"lower": [6, 139, 74], "upper":[24, 255, 161]},
                                 "green":{"lower": [56, 74, 18], "upper":[79, 255, 147]} ,
                                 "red": {"lower": [0, 245, 64], "upper":[180, 255, 143]},
                                 "brown": {"lower": [0, 59, 0], "upper":[38, 255, 55]} ,
                                 "grey": {"lower": [20, 29, 50], "upper":[64, 157, 108]},
                                 "black":{"lower": [0, 0, 0], "upper":[161, 255, 62]}
                                }

    def get_boundaries_dict(self):
        return self.color_boundaries
