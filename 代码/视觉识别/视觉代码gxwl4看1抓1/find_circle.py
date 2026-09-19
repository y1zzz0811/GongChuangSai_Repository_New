import cv2 as cv
import numpy as np
Colors_HSV = {
    "red": {"Min": np.array([150, 43, 46]), "Max": np.array([180, 255, 255])},
    "blue": {"Min": np.array([100, 43, 46]), "Max": np.array([124, 255, 255])},
    "green": {"Min": np.array([25, 30, 45]), "Max": np.array([85, 255, 255])},
    "general": {"Min": np.array([0, 30, 0]), "Max": np.array([180, 255, 255])},
}

class Find_circle():
    def __init__(self):
        self.PI = np.pi

    def find_circle_Pos(self, img, color, debug=False):
        hsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
        # hsv = cv.GaussianBlur(hsv, (5, 5), 0)
        dst = cv.inRange(hsv, Colors_HSV[color]["Min"], Colors_HSV[color]["Max"])
        # kernel = np.ones((3, 3), np.uint8)
        # dst = cv.erode(dst,kernel,iterations = 1)
        # dst = cv.dilate(dst, kernel, iterations=1)
        kernel = np.ones((3, 3), np.uint8)
        dst = cv.dilate(dst, kernel, iterations=7)
        dst = cv.erode(dst,kernel,iterations = 8)
        dst = cv.dilate(dst, kernel, iterations=1)
        contours, _ = cv.findContours(dst, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        max_area = 0
        max_contour = None
        circle_center = None

        for cnt in contours:
            if len(cnt) < 5:
                continue

            area = cv.contourArea(cnt)
            if  abs(area) > 50 and area > max_area:#abs(area) > 5000 and
                max_area = area
                max_contour = cnt
        if max_contour is not None:
            ellipse = cv.fitEllipse(max_contour)
            (x, y), _ , angle = ellipse
            # (x, y), _ = cv.minEnclosingCircle(max_contour)
            circle_center = (int(x), int(y))
            img = cv.ellipse(img, ellipse, (0, 255, 0), 2)
        # if debug and circle_center:
        #     cv.circle(img, (int(x), int(y)), 2, (0, 0, 255), 2, 8, 0)

        return circle_center

    def find_circle_Seq(self, img, debug=False):
        hsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 0)

        x_coords = {}  

        # 检测红色
        dst = cv.inRange(hsv, Colors_HSV["red"]["Min"], Colors_HSV["red"]["Max"])
        cnts_red, _ = cv.findContours(dst, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if cnts_red:
            c = max(cnts_red, key=cv.contourArea)
            area = cv.contourArea(c)
            if area >= 300:
                x_red = cv.boundingRect(c)[0]
                x_coords["red"] = x_red

        # 检测蓝色
        dst = cv.inRange(hsv, Colors_HSV["blue"]["Min"], Colors_HSV["blue"]["Max"])
        cnts_blue, _ = cv.findContours(dst, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if cnts_blue:
            c = max(cnts_blue, key=cv.contourArea)
            area = cv.contourArea(c)
            if area >= 300:
                x_blue = cv.boundingRect(c)[0]
                x_coords["blue"] = x_blue

        # 检测绿色
        dst = cv.inRange(hsv, Colors_HSV["green"]["Min"], Colors_HSV["green"]["Max"])
        cnts_green, _ = cv.findContours(dst, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if cnts_green:
            c = max(cnts_green, key=cv.contourArea)
            area = cv.contourArea(c)
            if area >= 300:
                x_green = cv.boundingRect(c)[0]
                x_coords["green"] = x_green

        # 如果只检测到两种颜色，将未检测到的颜色X坐标设为最小
        if len(x_coords) == 2:
            if "red" not in x_coords:
                x_coords["red"] = -1e6
            if "blue" not in x_coords:
                x_coords["blue"] = -1e6
            if "green" not in x_coords:
                x_coords["green"] = -1e6
        else:
            return None
        # 根据X坐标从小到大排序颜色
        sorted_colors = sorted(x_coords.keys(), key=lambda color: x_coords[color])

        Seq = ""
        for color in sorted_colors:
            if color == "red":
                Seq += "1"
            elif color == "green":
                Seq += "2"
            elif color == "blue":
                Seq += "3"
        return Seq