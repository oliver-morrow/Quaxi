# implementation of canny edge detection for lane following
import cv2
import matplotlib.pyplot as plt
import numpy as np

def grey(image):
  #convert to grayscale
    image = np.asarray(image)
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def gauss(image):
    return cv2.GaussianBlur(image, (5, 5), 0)

def canny(image):
    edges = cv2.Canny(image,50,150)
    return edges

def region(image):
    height, width = image.shape
    triangle = np.array([
                       [[0, height], [2000,1000], [width, height]]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return lines_image

def average(image, lines):
    left = []
    right = []

    if lines is not None:
      for line in lines:
        print(line)
        x1, y1, x2, y2 = line.reshape(4)

        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        print(parameters)
        slope = parameters[0]
        y_int = parameters[1]

        if slope < 0:
            left.append((slope, y_int))
        else:
            right.append((slope, y_int))
            
    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)

    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

def make_points(image, average):
    print(average)
    slope, y_int = average
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - y_int) // slope)
    x2 = int((y2 - y_int) // slope)
    return np.array([x1, y1, x2, y2])

def init_cv(img_dir):
    current_frame = cv2.imread("test.jpg") # replace with current_frame = cv2.imread(img_dir) later
    current_frame_greyscale = grey(current_frame)

    current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_RGB2HSV)
    lower_yellow = np.array([10,100,100], dtype = "uint8")
    upper_yellow = np.array([100,255,255], dtype = "uint8")
    mask_yellow = cv2.inRange(current_frame_hsv, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(current_frame_greyscale, 200, 255)
    mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    mask_yw_image = cv2.bitwise_and(current_frame_greyscale, mask_yw)

    frame_copy = np.copy(mask_yw_image)
    frame_edges = cv2.Canny(frame_copy, 50, 150)
    isolated = region(frame_edges)
    cv2.waitKey(0)

    lines = cv2.HoughLinesP(isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = average(frame_copy, lines)
    black_lines = display_lines(frame_copy, averaged_lines)

    lanes = cv2.addWeighted(frame_copy, 0.8, black_lines, 1, 1)
    return lanes