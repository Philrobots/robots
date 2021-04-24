import rospy
import RPi.GPIO as GPIO

from capture_image_from_embed_camera import capture_image_from_embed_camera
from map_letters import map_letters
from process_image_to_grayscale import process_image_to_grayscale


class Mapping:

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(11, GPIO.OUT)
        self.servo_y = GPIO.PWM(11, 50)
        GPIO.setup(12, GPIO.OUT)
        self.servo_x = GPIO.PWM(12, 50)
        self.initialized = False

    def start_y_servo(self):
        self.servo_y.start(0)
        if not self.initialized:
            self.servo_y.ChangeDutyCycle(7.5)
            self.initialized = True
        rospy.sleep(1)
        self.servo_y.stop()
        self.servo_x.start(0)

    def letter_mapping(self):
        servo_x_angle = [5, 6.5, 7, 8, 9, 10]
        self.start_y_servo()
        rospy.sleep(1)
        for angle in servo_x_angle:
            letters = self.camera_panning(angle)
            if len(letters) == 9:
                return letters
        return []

    def camera_panning(self, x_position):

        self.servo_x.ChangeDutyCycle(x_position)
        rospy.sleep(0.5)

        image = capture_image_from_embed_camera()
        grayscale = process_image_to_grayscale(image)
        letters = map_letters(grayscale)
        return letters


if __name__ == "__main__":

    mapping = Mapping()
    while True:
        mapping.letter_mapping()
        input()
