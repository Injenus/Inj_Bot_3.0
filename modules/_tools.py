import cv2
import numpy as np

def resize(koeff, frame):
    assert frame is not None
    return cv2.resize(frame, (round(frame.shape[1]/koeff), round(frame.shape[0]/koeff)))


class Color_Mask_HSV():
    def __init__(self, h_min, h_max, s_min, s_max, v_min, v_max):
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        self.mins = [self.h_min, self.s_min, self.v_min]
        self.maxs = [self.h_max, self.s_max, self.v_max]


class Color_Mask_LAB():
    def __init__(self, l_min, l_max, a_min, a_max, b_min, b_max):
        self.l_min = l_min
        self.l_max = l_max
        self.a_min = a_min
        self.a_max = a_max
        self.b_min = b_min
        self.b_max = b_max
        self.mins = [self.l_min, self.a_min, self.b_min]
        self.maxs = [self.l_max, self.a_max, self.b_max]


def detect_largest_region(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return -1, -1, -1, (-1,-1,-1)

    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)

    moments = cv2.moments(largest_contour)
    if moments["m00"] != 0:
        center_x = round(moments["m10"] / moments["m00"])
        center_y = round(moments["m01"] / moments["m00"])
    else:
        center_x, center_y = -1, -1
    (x,y),radius = cv2.minEnclosingCircle(largest_contour)
    return center_x, center_y, area, (round(x),round(y),round(radius))     


def detect_shapes(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_triangle_area = 0
    max_rectangle_area = 0
    largest_triangle = None
    largest_rectangle = None

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 3:  # Проверка на треугольник
            area = cv2.contourArea(approx)
            if area > max_triangle_area:
                max_triangle_area = area
                largest_triangle = approx

        elif len(approx) == 4:  # Проверка на прямоугольник
            points = approx.reshape(4, 2)

            def angle(p1, p2, p3):
                v1 = p1 - p2
                v2 = p3 - p2
                dot = np.dot(v1, v2)
                mag1 = np.linalg.norm(v1)
                mag2 = np.linalg.norm(v2)
                cos_angle = dot / (mag1 * mag2)
                return np.degrees(np.arccos(cos_angle))

            angles = [
                angle(points[i], points[(i + 1) % 4], points[(i + 2) % 4]) for i in range(4)
            ]
            if all(80 <= a <= 100 for a in angles):  # Проверка углов (близость к 90°)
                def distance(p1, p2):
                    return np.linalg.norm(p1 - p2)

                d1 = distance(points[0], points[1])
                d2 = distance(points[1], points[2])
                d3 = distance(points[2], points[3])
                d4 = distance(points[3], points[0])

                if abs(d1 - d3) < 0.1 * max(d1, d3) and abs(d2 - d4) < 0.1 * max(d2, d4):
                    # Это прямоугольник
                    area = cv2.contourArea(approx)
                    if area > max_rectangle_area:
                        max_rectangle_area = area
                        largest_rectangle = approx

    def get_counter_data(contour, area):
        if contour is not None:
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                center_x = round(moments["m10"] / moments["m00"])
                center_y = round(moments["m01"] / moments["m00"])
            else:
                center_x, center_y = -1, -1
            (x,y),radius = cv2.minEnclosingCircle(contour)
            return center_x, center_y, area, (round(x), round(y), round(radius))
        return -1, -1, -1, (-1, -1, -1)
    
    return get_counter_data(largest_rectangle, max_rectangle_area), get_counter_data(largest_triangle, max_triangle_area)



def simple_ctrl(err):
    assert err < 1000 and err > -1000

    err = err/1000
    k = 1
    if err < 0:
        k = -1

    delta = 6*k * err**2

    return delta
