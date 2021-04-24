from collections import Counter


import cv2
import numpy as np

from sklearn.cluster import KMeans
from scripts.src.detection.color_boundaries import ColorBoundaries


class PuckDetection:

    def __init__(self):
        self.color_boundaries = ColorBoundaries()

    @staticmethod
    def copy_image(image):
        try:
            img = image.copy()
        except AttributeError as invalid_image:
            raise AttributeError("L'image est invalide") from invalid_image
        return img


    def detect_pucks(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        image = self.remove_glare(image)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 10, param1=50,
                                   param2=30, minRadius=23, maxRadius=30)
        detected_circles = np.uint16(np.around(circles))

        puck_positions = {}

        for (x, y, radius) in detected_circles[0].astype(np.int32):
            roi = image[y - radius: y + radius, x - radius: x + radius]
            width, height = roi.shape[:2]
            mask = np.zeros((width, height, 3), roi.dtype)

            cv2.circle(mask, (int(width / 2), int(height / 2)), radius,
                       (255, 255, 255), -1)

            dominant_color = self.get_dominant_color(roi, k=4)

            dominant_color_np = np.uint8([[dominant_color]])
            hsv_dominant_color = cv2.cvtColor(dominant_color_np, cv2.COLOR_BGR2HSV)
            hsv_color = self.find_hsv_color(hsv_dominant_color[0][0])

            puck_positions[hsv_color] = [] if puck_positions.get(hsv_color) is None else puck_positions.get(hsv_color)
            puck = dict()
            puck["center_position"] = (x, y)
            puck["radius"] = radius
            puck_positions[hsv_color].append(puck)

        return puck_positions

    def remove_glare(self, image):
        GLARE_MIN = np.array([0, 0, 20], np.uint8)
        GLARE_MAX = np.array([0, 0, 255], np.uint8)

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        frame_threshed = cv2.inRange(hsv_img, GLARE_MIN, GLARE_MAX)
        result = cv2.inpaint(image, frame_threshed, 0.6, cv2.INPAINT_TELEA)

        image_lab = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
        image_lab_planes = cv2.split(image_lab)
        image_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        image_lab_planes[0] = image_clahe.apply(image_lab_planes[0])
        image_lab = cv2.merge(image_lab_planes)

        image = cv2.cvtColor(image_lab, cv2.COLOR_LAB2BGR)
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype("float32")
        (h, s, v) = cv2.split(image_hsv)
        s = s * 2
        s = np.clip(s, 0, 255)
        image_hsv = cv2.merge([h, s, v])
        image = cv2.cvtColor(image_hsv.astype("uint8"), cv2.COLOR_HSV2BGR)
        hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsvImg[..., 2] = hsvImg[..., 2] * 0.85
        image = cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)
        return image

    def show_image(self, output):
        cv2.imshow('output', output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def draw_on_image(self, hsv_color, output, r, x, y):
        cv2.circle( output, (x, y), r, (0, 255, 0), 2 )
        cv2.circle( output, (x, y), 2, (0, 255, 255), 2 )
        cv2.putText( output, "{}".format( hsv_color ), (x - 30, y - 30),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1 )
        cv2.putText( output, "({}, {})".format( x, y ), (x, y),
                     cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1 )

    def get_dominant_color(self, image, k=4, image_processing_size=None):
        if image_processing_size is not None:
            image = cv2.resize( image, image_processing_size,
                                interpolation=cv2.INTER_AREA)

        image = image.reshape( (image.shape[0] * image.shape[1], 3) )

        clt = KMeans(n_clusters=k)
        labels = clt.fit_predict(image)
        label_counts = Counter(labels)

        dominant_color = clt.cluster_centers_[label_counts.most_common(1)[0][0]]
        return list(dominant_color)

    def find_hsv_color(self, hsv):
        colors = self.color_boundaries.get_boundaries_dict()
        for color, boundaries in colors.items():
            if boundaries["lower"][0] <= hsv[0] <= boundaries["upper"][0] and \
                    boundaries["lower"][1] <= hsv[1] <= \
                    boundaries["upper"][1] and boundaries["lower"][2] <= hsv[2] \
                    <= boundaries["upper"][2]:
                return color
        return "None"
