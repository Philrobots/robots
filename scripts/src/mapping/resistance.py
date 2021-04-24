import math

from scripts.src.mapping.resistance_value import ResistanceValue


class Resistance:
    """Class used to encapsulate resistances and then allow us to be able to
    extract useful information for the resistance color code sheet"""

    def __init__(self, resistance):
        self.resistance = resistance

    def get_resistance_and_colors(self):
        log10 = max(0, math.floor(math.log10(self.resistance)-1))
        resistance_values = [(resistance[0]*(pow(10, log10)), resistance[1]) for resistance in ResistanceValue().RESISTANCE_VALUES] + [(resistance[0]*(pow(10, log10+1)), resistance[1]) for resistance in ResistanceValue().RESISTANCE_VALUES]
        distances = [(i, abs(self.resistance - model_resistance[0]), model_resistance[1]) for i, model_resistance in enumerate(resistance_values)]

        closest_index = min(distances, key=lambda x: x[1])[0]
        closest_resistance, colors = resistance_values[closest_index]
        return closest_resistance, colors

    def find_first_digits(self):
        resistance, _ = self.get_resistance_and_colors()
        resistance = resistance * 100
        return [int(digit) for digit in str(resistance)[:2]]

    def get_first_digit(self):
        return self.find_first_digits()[0]

    def get_second_digit(self):
        return self.find_first_digits()[1]
