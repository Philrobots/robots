import cv2


def capture_image_from_embed_camera():
    capture = cv2.VideoCapture(0)

    if not capture.isOpened():
        raise Exception('Could not open camera')

    _, frame = capture.read()

    capture.release()
    cv2.destroyAllWindows()  # TODO : Remove it or not ?

    return frame
