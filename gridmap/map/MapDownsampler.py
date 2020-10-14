#!/usr/bin/python
#-*- encoding: utf8 -*-

import scipy.misc
import cv2
import numpy as np


if __name__ == '__main__':
    df = 5 # Downsampling factor

    map_1_ = np.load('map_processed_0.1.npy').astype(np.uint8)
    map_1_[map_1_>0] = 1

    new_shape = np.array(map_1_.shape, dtype = 'float')
    new_shape = new_shape / df
    new_shape = new_shape.astype('int')
    new_mat = np.zeros(new_shape, dtype = np.uint8)

    for i in range(new_shape[0]):
        for j in range(new_shape[1]):
            block = map_1_[(df*i):(df*i+df),(df*j):(df*j+df)]
            if float(sum(sum(block))) > 0.3*(df*df):
                new_mat[i][j] = 255
    
    np.save('map_processed_0.5.npy', new_mat)
    scipy.misc.toimage(new_mat, cmin=0, cmax=255).save('map_processed_0.5.jpg')

