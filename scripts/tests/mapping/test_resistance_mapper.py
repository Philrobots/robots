from unittest.mock import Mock
import pytest

from scripts.src.mapping.resistance_mapper import ResistanceMapper
from scripts.src.mapping.color import Color


class TestResistanceMapper:
    """Test ResistanceMapper class"""
    @classmethod
    def setup_class(cls):
        cls.FIRST_DIGIT_1 = 3
        cls.FIRST_DIGIT_2 = 4
        cls.SECOND_DIGIT_1 = 5
        cls.SECOND_DIGIT_2 = 7
        cls.EXPONENT_1 = 3
        cls.EXPONENT_2 = 6
        cls.UNEXPECTED_DIGIT = -1

        cls.EXPECTED_COLORS_1 = [Color.ORANGE, Color.GREEN, Color.ORANGE]
        cls.EXPECTED_COLORS_2 = [Color.YELLOW, Color.VIOLET, Color.BLUE]

    def setup_method(self):
        self.resistance_mapper = ResistanceMapper()
        self.resistance = Mock()

    def test_when_find_colors_then_resistance_finds_its_first_digit(self):
        self.resistance.get_first_digit.return_value = self.FIRST_DIGIT_1
        self.resistance.get_second_digit.return_value = self.SECOND_DIGIT_1
        self.resistance.get_exponent.return_value = self.EXPONENT_1

        self.resistance_mapper.find_colors(self.resistance)

        self.resistance.get_first_digit.assert_called_once()

    def test_when_find_colors_then_resistance_finds_its_second_digit(self):
        self.resistance.get_first_digit.return_value = self.FIRST_DIGIT_1
        self.resistance.get_second_digit.return_value = self.SECOND_DIGIT_1
        self.resistance.get_exponent.return_value = self.EXPONENT_1

        self.resistance_mapper.find_colors(self.resistance)

        self.resistance.get_second_digit.assert_called_once()

    def test_when_find_colors_then_resistance_finds_its_exponent(self):
        self.resistance.get_first_digit.return_value = self.FIRST_DIGIT_1
        self.resistance.get_second_digit.return_value = self.SECOND_DIGIT_1
        self.resistance.get_exponent.return_value = self.EXPONENT_1

        self.resistance_mapper.find_colors(self.resistance)

        self.resistance.get_exponent.assert_called_once()

    def test_when_find_colors_then_return_expected_colors_1(self):
        self.resistance.get_first_digit.return_value = self.FIRST_DIGIT_1
        self.resistance.get_second_digit.return_value = self.SECOND_DIGIT_1
        self.resistance.get_exponent.return_value = self.EXPONENT_1

        colors = self.resistance_mapper.find_colors(self.resistance)

        assert colors == self.EXPECTED_COLORS_1

    def test_when_find_colors_then_return_expected_colors_2(self):
        self.resistance.get_first_digit.return_value = self.FIRST_DIGIT_2
        self.resistance.get_second_digit.return_value = self.SECOND_DIGIT_2
        self.resistance.get_exponent.return_value = self.EXPONENT_2

        colors = self.resistance_mapper.find_colors(self.resistance)

        assert colors == self.EXPECTED_COLORS_2

    def test_given_unexpected_digit_when_find_colors_then_throw_exception(self):
        self.resistance.get_first_digit.return_value = self.UNEXPECTED_DIGIT
        with pytest.raises(Exception):
            self.resistance_mapper.find_colors(self.resistance)
