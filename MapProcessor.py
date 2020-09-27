#!/usr/bin/python
#-*- encoding: utf8 -*-

import scipy.misc
import cv2
import numpy as np


if __name__ == '__main__':
    map_1_ = np.load('map_1.npy').astype(np.uint8)
    map_1_[map_1_>0] = 255
    #cv2.imshow('GrayImage', map_1_)
    #cv2.waitKey()

    ret, binary = cv2.threshold(map_1_, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dst = cv2.dilate(binary, kernel)
    #cv2.imshow("dilate", dst)
    #cv2.waitKey()

    filled = dst.copy()
    (h,w) = map_1_.shape
    mask = np.zeros([h+2, w+2], np.uint8)
    cv2.floodFill(filled, mask, (1,1), 100, 0, 0)
    #cv2.imshow("filled", filled)
    #cv2.waitKey()

    filled[filled!=100] = 255
    filled[filled==100] = 0
    cv2.imshow("filled_modified", filled)
    cv2.waitKey()
    
    np.save('map_processed.npy', filled)
    scipy.misc.toimage(filled, cmin=0, cmax=255).save('map_processed.jpg')

