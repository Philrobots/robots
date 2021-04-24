from scripts.src.mapping.letter import Letter


class TestCorner:
    def test_given_letter_a_when_get_next_then_get_letter_b(self):
        letter = Letter.A

        next_letter = letter.get_next_letter()

        assert next_letter is Letter.B

    def test_given_letter_b_when_get_next_then_get_letter_c(self):
        letter = Letter.B

        next_letter = letter.get_next_letter()

        assert next_letter is Letter.C

    def test_given_letter_c_when_get_next_then_get_letter_d(self):
        letter = Letter.C

        next_letter = letter.get_next_letter()

        assert next_letter is Letter.D

    def test_given_letter_d_when_get_next_then_get_letter_a(self):
        letter = Letter.D

        next_letter = letter.get_next_letter()

        assert next_letter is Letter.A
