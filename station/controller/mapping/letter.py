from enum import Enum


class Letter(Enum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"

    def get_next_letter(self):
        next_letter = {
            Letter.A: Letter.B,
            Letter.B: Letter.C,
            Letter.C: Letter.D,
            Letter.D: Letter.A
        }
        return next_letter[self]
