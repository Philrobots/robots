import cv2


def process_image_to_grayscale(image):
    _, grayscale = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
    #to try gray=cv2.cvtColor(images, cv2.COLOR_BGR2GRAY)
    return grayscale
