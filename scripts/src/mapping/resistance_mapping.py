# résistances entre 10^2 et 10^6 ohms
import argparse

from scripts.src.mapping.resistance import Resistance
from scripts.src.mapping.resistance_mapper import ResistanceMapper

if __name__ == '__main__':
    AP = argparse.ArgumentParser()
    AP.add_argument("-r", "--resistance", help="path to the image")
    ARGS = vars(AP.parse_args())

    resistance = Resistance(int(ARGS["resistance"]))

    resistance_mapper = ResistanceMapper()

    print(resistance_mapper.find_colors(resistance))
