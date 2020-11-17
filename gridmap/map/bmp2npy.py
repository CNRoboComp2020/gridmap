#!/usr/bin/python
#-*- encoding: utf8 -*-

import scipy.misc
import cv2
import sys
import numpy as np


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Error: len(sys.argv) must be 2.')
        exit()
    bmp_name = str(sys.argv[1])
    bmp = cv2.imread(bmp_name)
    bmp = cv2.cvtColor(bmp, cv2.COLOR_BGR2GRAY)
    map_1_ = np.array(bmp, dtype = np.uint8)
    map_1_[map_1_>0] = 255
    np.save(bmp_name[:-4]+'.npy', map_1_)

