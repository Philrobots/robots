class Corner:
    def __init__(self, letter, position):
        self.letter = letter
        self.position = position
        self.color = None

    def set_color(self, color):
        self.color = color
