import cv2
import numpy as np
import stack_images


class TrackBarDetection:

    def on_track_bar_change(self, value):
        print(self, value)

    def start_track_bar(self):
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars", 640, 240)
        cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, self.on_track_bar_change)
        cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, self.on_track_bar_change)
        cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, self.on_track_bar_change)
        cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, self.on_track_bar_change)
        cv2.createTrackbar("Val Min", "TrackBars", 0, 255, self.on_track_bar_change)
        cv2.createTrackbar("Val Max", "TrackBars", 255, 255, self.on_track_bar_change)

        image_camera_monde = cv2.imread("trajectory_example_3.jpg")

        GLARE_MIN = np.array([0, 0, 20],np.uint8)
        GLARE_MAX = np.array([0, 0, 255],np.uint8)

        hsv_img = cv2.cvtColor(image_camera_monde,cv2.COLOR_BGR2HSV)

        frame_threshed = cv2.inRange(hsv_img, GLARE_MIN, GLARE_MAX)

        result = cv2.inpaint(image_camera_monde, frame_threshed, 0.6, cv2.INPAINT_TELEA)
        lab1 = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
        lab_planes1 = cv2.split(lab1)
        clahe1 = cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8))
        lab_planes1[0] = clahe1.apply(lab_planes1[0])
        lab1 = cv2.merge(lab_planes1)
        image = cv2.cvtColor(lab1, cv2.COLOR_LAB2BGR)

        imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype("float32")
        (h, s, v) = cv2.split(imghsv)
        s = s*2
        s = np.clip(s,0,255)
        imghsv = cv2.merge([h,s,v])

        image = cv2.cvtColor(imghsv.astype("uint8"), cv2.COLOR_HSV2BGR)

        hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        hsvImg[...,2] = hsvImg[...,2]*0.8
        image_camera_monde=cv2.cvtColor(hsvImg,cv2.COLOR_HSV2BGR)

        while True:
            hsv_image = cv2.cvtColor(image_camera_monde, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
            h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
            s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
            s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
            v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
            v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

            print(h_min, s_min, v_min, h_max, s_max, v_max)

            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv_image, lower, upper)

            image_result = cv2.bitwise_and(image_camera_monde, image_camera_monde, mask=mask)

            image_stack = stack_images.stackImages(0.6, ([image_camera_monde, hsv_image], [mask, image_result]))

            cv2.imshow("Image hsv", image_stack)

            cv2.waitKey(2000)


trackBar = TrackBarDetection()
trackBar.start_track_bar()
