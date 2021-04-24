from mapping.color import Color


class ResistanceMapper:
    """Class used to map resistances to colors"""
    def __init__(self):
        self.number_to_color = {
            0: Color.BLACK.value,
            1: Color.BROWN.value,
            2: Color.RED.value,
            3: Color.ORANGE.value,
            4: Color.YELLOW.value,
            5: Color.GREEN.value,
            6: Color.BLUE.value,
            7: Color.VIOLET.value,
            8: Color.GREY.value,
            9: Color.WHITE.value
        }

    def find_exponent_color(self, resistance):
        return self.number_to_color[resistance.get_exponent()]
