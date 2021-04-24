from scripts.src.mapping.command_panel import CommandPanel
from scripts.src.mapping.letter import Letter


class TestCommandPanel:
    def setup_method(self):
        self.command_panel = CommandPanel()

    def test_when_find_first_corner_letter_1_then_return_correct_letter(self):
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        resistance = 1234
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_resistance(resistance)

        letter = self.command_panel.find_first_corner_letter()

        assert letter is Letter.A

    def test_when_find_first_corner_letter_2_then_return_correct_letter(self):
        mapped_letters = ["A", "B", "B", "D", "B", "B", "B", "B", "B"]
        resistance = 4567
        self.command_panel.set_mapped_letters(mapped_letters)
        self.command_panel.set_resistance(resistance)

        letter = self.command_panel.find_first_corner_letter()

        assert letter is Letter.D
