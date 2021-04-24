from scripts.src.mapping.letter import Letter
from scripts.src.mapping.resistance import Resistance
from scripts.src.mapping.resistance_mapper import ResistanceMapper


class CommandPanel:
    def __init__(self):
        self.resistance = None
        self.mapped_letters = None
        self.resistance_mapper = ResistanceMapper()

    def set_resistance(self, resistance):
        self.resistance = Resistance(resistance)

    def set_mapped_letters(self, mapped_letters):
        self.mapped_letters = mapped_letters

    def find_first_corner_letter(self):
        first_digit = self.resistance.get_first_digit()
        index = first_digit - 1
        return Letter[self.mapped_letters[index]]
