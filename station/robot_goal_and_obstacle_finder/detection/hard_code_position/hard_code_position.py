import cv2


def position_of_resistance_panel():
    position_x_of_resistance_panel = 150
    position_y_of_resistance_panel = 750
    return position_x_of_resistance_panel, position_y_of_resistance_panel


def position_of_control_panel():
    position_x_of_control_panel = 1300
    position_y_of_control_panel = 430
    return position_x_of_control_panel, position_y_of_control_panel


def test_hard_code_position(image):
    image = cv2.imread(image)
    print(image.shape)
    position = position_of_control_panel()
    cv2.putText(image, "X", position,
                cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)
    cv2.imshow('square detection', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
