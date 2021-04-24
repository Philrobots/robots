from scripts.src.mapping.color import Color


class ResistanceMapper:
    """Class used to map resistances to colors"""
    def __init__(self):
        self.number_to_color = {
            0: Color.BLACK,
            1: Color.BROWN,
            2: Color.RED,
            3: Color.ORANGE,
            4: Color.YELLOW,
            5: Color.GREEN,
            6: Color.BLUE,
            7: Color.VIOLET,
            8: Color.GREY,
            9: Color.WHITE
        }

    def find_colors(self, resistance):
        return [
            self.number_to_color[resistance.get_first_digit()],
            self.number_to_color[resistance.get_second_digit()],
            self.number_to_color[resistance.get_exponent()]
        ]
