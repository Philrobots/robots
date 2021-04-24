import RPi.GPIO as GPIO

class RedLightDriver:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        self.LED = 15

        GPIO.setup(self.LED, GPIO.OUT)

    def on(self):
        GPIO.output(self.LED, GPIO.HIGH)

    def off(self):
        GPIO.output(self.LED, GPIO.LOW)


if __name__ == "__main__":
    s = RedLightDriver()
    s.off()
