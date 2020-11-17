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
    npy_name = str(sys.argv[1])
    map_1_ = np.load(npy_name).astype(np.uint8)
    map_1_[map_1_>0] = 255
    scipy.misc.toimage(map_1_, cmin=0, cmax=255).save(npy_name[:-4]+'.bmp')

