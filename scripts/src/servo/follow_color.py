import RPi.GPIO as GPIO
import cv2

from scripts.src.detection.lower_boundary import LowerBoundary
from scripts.src.detection.upper_boundary import UpperBoundary
from scripts.src.mapping.color import Color

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
servo1 = GPIO.PWM(11, 50)
GPIO.setup(12, GPIO.OUT)
servo2 = GPIO.PWM(12, 50)

servo1.start(0)
servo2.start(0)

cap = cv2.VideoCapture(0)
CAP_WIDTH = 480
CAP_HEIGHT = 320
cap.set(3, CAP_WIDTH)
cap.set(4, CAP_HEIGHT)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
x_center = int(cols / 2)
y_medium = int(rows / 2)
y_center = int(rows / 2)
x_position = 90
y_position = 90

MOVEMENT_THRESHOLD = 30


def angle_to_per(angle):
    return 2+(angle/18)


def contour_area(contour):
    return cv2.contourArea(contour)


def adjust_position_for_x(original, medium, center):
    if original > 180 or original < 0:
        return original

    position = original

    if medium < center - MOVEMENT_THRESHOLD:
        position -= 1
    elif medium > center + MOVEMENT_THRESHOLD:
        position += 1

    return position


def adjust_position_for_y(original, medium, center):
    if original > 180 or original < 0:
        return original

    position = original

    if medium < center - MOVEMENT_THRESHOLD:
        position += 1
    elif medium > center + MOVEMENT_THRESHOLD:
        position -= 1

    return position


while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # TODO : Could we receive another color?
    lower_color = LowerBoundary().get_lower_boundaries(Color.RED)
    upper_color = UpperBoundary().get_upper_boundaries(Color.RED)
    color_mask = cv2.inRange(hsv_frame, lower_color, upper_color)
    _, contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=contour_area, reverse=True)

    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)

        x_medium = int((x + x + w) / 2)
        y_medium = int((y + y + h) / 2)
        break

    x_position = adjust_position_for_x(x_position, x_medium, x_center)
    if(x_medium < x_center - MOVEMENT_THRESHOLD or x_medium > x_center + MOVEMENT_THRESHOLD):
        servo2.ChangeDutyCycle(angle_to_per(x_position))

    y_position = adjust_position_for_y(y_position, y_medium, y_center)
    if(y_medium < y_center - MOVEMENT_THRESHOLD or y_medium > y_center + MOVEMENT_THRESHOLD):
        servo1.ChangeDutyCycle(angle_to_per(y_position))

    cv2.line(frame, (x_medium, 0), (x_medium, CAP_WIDTH), (0, 255, 0), 2)
    cv2.line(frame, (0, y_medium), (CAP_WIDTH, y_medium), (0, 255, 0), 2)
    cv2.imshow("Frame", frame)

    #serial.write(x_position)
    print(x_position)

    if cv2.waitKey(1) == ord('a'):
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()
        break

cap.release()
cv2.destroyAllWindows()
