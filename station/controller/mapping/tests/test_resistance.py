from mapping.resistance import Resistance


class TestResistance:
    """Test Resistance class"""
    @classmethod
    def setup_class(cls):
        cls.A_RESISTANCE_1 = 950
        cls.A_RESISTANCE_2 = 1050
        cls.A_RESISTANCE_3 = 1150
        cls.A_RESISTANCE_4 = 1250
        cls.A_RESISTANCE_5 = 1300
        cls.A_RESISTANCE_6 = 1400
        cls.A_RESISTANCE_7 = 1700
        cls.A_RESISTANCE_8 = 1900
        cls.A_RESISTANCE_9 = 2100
        cls.A_RESISTANCE_10 = 2300
        cls.A_RESISTANCE_11 = 2600
        cls.A_RESISTANCE_12 = 2800
        cls.A_RESISTANCE_13 = 3300
        cls.A_RESISTANCE_14 = 3800
        cls.A_RESISTANCE_15 = 4000
        cls.A_RESISTANCE_16 = 4500
        cls.A_RESISTANCE_17 = 4800
        cls.A_RESISTANCE_18 = 5500
        cls.A_RESISTANCE_19 = 6000
        cls.A_RESISTANCE_20 = 6600
        cls.A_RESISTANCE_21 = 7000
        cls.A_RESISTANCE_22 = 8000
        cls.A_RESISTANCE_23 = 8500
        cls.A_RESISTANCE_24 = 9999
        cls.A_RESISTANCE_25 = 10500
        cls.A_RESISTANCE_26 = 34000
        cls.A_RESISTANCE_27 = 1330

        cls.EXPECTED_RESISTANCE_1 = 1000
        cls.EXPECTED_RESISTANCE_2 = 1000
        cls.EXPECTED_RESISTANCE_3 = 1200
        cls.EXPECTED_RESISTANCE_4 = 1200
        cls.EXPECTED_RESISTANCE_5 = 1200
        cls.EXPECTED_RESISTANCE_6 = 1500
        cls.EXPECTED_RESISTANCE_7 = 1800
        cls.EXPECTED_RESISTANCE_8 = 1800
        cls.EXPECTED_RESISTANCE_9 = 2200
        cls.EXPECTED_RESISTANCE_10 = 2200
        cls.EXPECTED_RESISTANCE_11 = 2700
        cls.EXPECTED_RESISTANCE_12 = 2700
        cls.EXPECTED_RESISTANCE_13 = 3300
        cls.EXPECTED_RESISTANCE_14 = 3900
        cls.EXPECTED_RESISTANCE_15 = 3900
        cls.EXPECTED_RESISTANCE_16 = 4700
        cls.EXPECTED_RESISTANCE_17 = 4700
        cls.EXPECTED_RESISTANCE_18 = 5600
        cls.EXPECTED_RESISTANCE_19 = 5600
        cls.EXPECTED_RESISTANCE_20 = 6800
        cls.EXPECTED_RESISTANCE_21 = 6800
        cls.EXPECTED_RESISTANCE_22 = 8200
        cls.EXPECTED_RESISTANCE_23 = 8200
        cls.EXPECTED_RESISTANCE_24 = 10000
        cls.EXPECTED_RESISTANCE_25 = 10000
        cls.EXPECTED_RESISTANCE_26 = 33000
        cls.EXPECTED_RESISTANCE_27 = 1200

        cls.EXPECTED_EXPONENT_1 = 1
        cls.EXPECTED_EXPONENT_2 = 2
        cls.EXPECTED_EXPONENT_3 = 3
        cls.EXPECTED_EXPONENT_4 = 4
        cls.EXPECTED_EXPONENT_5 = 5

    def test_when_round_then_resistance_is_as_expected_1(self):
        resistance = Resistance(self.A_RESISTANCE_1)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_1

    def test_when_round_then_resistance_is_as_expected_2(self):
        resistance = Resistance(self.A_RESISTANCE_2)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_2

    def test_when_round_then_resistance_is_as_expected_3(self):
        resistance = Resistance(self.A_RESISTANCE_3)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_3

    def test_when_round_then_resistance_is_as_expected_4(self):
        resistance = Resistance(self.A_RESISTANCE_4)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_4

    def test_when_round_then_resistance_is_as_expected_5(self):
        resistance = Resistance(self.A_RESISTANCE_5)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_5

    def test_when_round_then_resistance_is_as_expected_6(self):
        resistance = Resistance(self.A_RESISTANCE_6)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_6

    def test_when_round_then_resistance_is_as_expected_7(self):
        resistance = Resistance(self.A_RESISTANCE_7)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_7

    def test_when_round_then_resistance_is_as_expected_8(self):
        resistance = Resistance(self.A_RESISTANCE_8)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_8

    def test_when_round_then_resistance_is_as_expected_9(self):
        resistance = Resistance(self.A_RESISTANCE_9)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_9

    def test_when_round_then_resistance_is_as_expected_10(self):
        resistance = Resistance(self.A_RESISTANCE_10)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_10

    def test_when_round_then_resistance_is_as_expected_11(self):
        resistance = Resistance(self.A_RESISTANCE_11)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_11

    def test_when_round_then_resistance_is_as_expected_12(self):
        resistance = Resistance(self.A_RESISTANCE_12)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_12

    def test_when_round_then_resistance_is_as_expected_13(self):
        resistance = Resistance(self.A_RESISTANCE_13)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_13

    def test_when_round_then_resistance_is_as_expected_14(self):
        resistance = Resistance(self.A_RESISTANCE_14)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_14

    def test_when_round_then_resistance_is_as_expected_15(self):
        resistance = Resistance(self.A_RESISTANCE_15)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_15

    def test_when_round_then_resistance_is_as_expected_16(self):
        resistance = Resistance(self.A_RESISTANCE_16)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_16

    def test_when_round_then_resistance_is_as_expected_17(self):
        resistance = Resistance(self.A_RESISTANCE_17)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_17

    def test_when_round_then_resistance_is_as_expected_18(self):
        resistance = Resistance(self.A_RESISTANCE_18)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_18

    def test_when_round_then_resistance_is_as_expected_19(self):
        resistance = Resistance(self.A_RESISTANCE_19)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_19

    def test_when_round_then_resistance_is_as_expected_20(self):
        resistance = Resistance(self.A_RESISTANCE_20)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_20

    def test_when_round_then_resistance_is_as_expected_21(self):
        resistance = Resistance(self.A_RESISTANCE_21)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_21

    def test_when_round_then_resistance_is_as_expected_22(self):
        resistance = Resistance(self.A_RESISTANCE_22)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_22

    def test_when_round_then_resistance_is_as_expected_23(self):
        resistance = Resistance(self.A_RESISTANCE_23)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_23

    def test_when_round_then_resistance_is_as_expected_24(self):
        resistance = Resistance(self.A_RESISTANCE_24)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_24

    def test_when_round_then_resistance_is_as_expected_25(self):
        resistance = Resistance(self.A_RESISTANCE_25)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_25

    def test_when_round_then_resistance_is_as_expected_26(self):
        resistance = Resistance(self.A_RESISTANCE_26)
        rounded_resistance, _ = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_26

    def test_when_round_then_resistance_is_as_expected_27(self):
        resistance = Resistance(self.A_RESISTANCE_27)
        rounded_resistance, colors = resistance.get_resistance_and_colors()
        assert rounded_resistance == self.EXPECTED_RESISTANCE_27

    def test_when_find_exponent_then_exponent_is_as_expected_1(self):
        resistance = Resistance(100)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_1

    def test_when_find_exponent_then_exponent_is_as_expected_2(self):
        resistance = Resistance(1000)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_2

    def test_when_find_exponent_then_exponent_is_as_expected_3(self):
        resistance = Resistance(10000)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_3

    def test_when_find_exponent_then_exponent_is_as_expected_4(self):
        resistance = Resistance(100000)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_4

    def test_when_find_exponent_then_exponent_is_as_expected_5(self):
        resistance = Resistance(1000000)
        exponent = resistance.get_exponent()
        assert exponent == self.EXPECTED_EXPONENT_5
