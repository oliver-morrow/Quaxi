#!/user/bin/env python

"""camera_calibration.py: Camera calibration module.

Usage: camera_calibration.py
Ensure images of the chessboard are contained in the same folder. A camera_calibration.json file will be created which will contain the results of the calibration.
"""
__author__ = "Matthew Pan"
__copyright__ = "Copyright 2024, Matthew Pan"

import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import json
import matplotlib.cm as cm

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(obj)
    
CHESSBOARD = (6, 8)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

obj_points = []
img_points = []

objp = np.zeros((CHESSBOARD[0]*CHESSBOARD[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)

images = glob.glob('*.jpg')

for fname in images:
    print(fname)
    img