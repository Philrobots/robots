import argparse
import RPi.GPIO as GPIO

from scripts.src.capture.capture_image_from_embed_camera import capture_image_from_embed_camera
from scripts.src.capture.capture_image_from_path import capture_image_from_path
from scripts.src.mapping.map_letters import map_letters
from scripts.src.processing.process_image_to_grayscale import process_image_to_grayscale


GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
servo2 = GPIO.PWM(12, 50)

servo1.start(0)
servo2.start(0)
x_position = 7
y_position = 6
servo2.ChangeDutyCycle(x_position)
servo1.ChangeDutyCycle(y_position)

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", type=str, help="path to input image")
ap.add_argument("-e", "--embed-camera", type=bool,
                help="use embed camera to get image", default=False)
ap.add_argument("-p", "--path", type=str, help="path to tesseract.exe")
args = vars(ap.parse_args())

image = capture_image_from_embed_camera() \
    if args["embed-camera"] \
    else capture_image_from_path(args["image"])

grayscale = process_image_to_grayscale(image)

letters = map_letters(grayscale)
