import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

LED = 15

GPIO.setup(LED, GPIO.OUT)

state = GPIO.input(LED)
while True:
    GPIO.output(LED, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(LED, GPIO.LOW)
    time.sleep(3)
    