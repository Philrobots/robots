from mapping.letter import Letter
from mapping.resistance import Resistance


class CommandPanel:
    def __init__(self):
        self.resistance = None
        self.mapped_letters = None

    def set_coordinates(self, coordinates):
        self.coordinates = coordinates

    def set_resistance(self, resistance):
        self.resistance = Resistance(resistance) #TODO: s'assurer que Resistance(resistance) a le bon comportement (le truc de e12(?) de denis)

    def set_mapped_letters(self, mapped_letters):
        self.mapped_letters = mapped_letters

    def find_first_corner_letter(self):
        first_digit = self.resistance.get_first_digit()
        index = first_digit - 1
        return Letter[self.mapped_letters[index]]
