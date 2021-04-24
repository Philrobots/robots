import cv2

STOP_KEY = 'q'  # TODO : Is there a better key we could use?


# TODO : Test this (can we?)
def capture_image_from_embed_camera():
    capture = cv2.VideoCapture(0)

    if not capture.isOpened():
        raise Exception('Could not open camera')

    _, frame = capture.read()

    while True:
        cv2.imshow('preview', frame)
        if cv2.waitKey(1) & 0xFF == ord(STOP_KEY):
            break

    capture.release()
    cv2.destroyAllWindows()

    return frame
