#!/usr/bin/python
#-*- encoding: utf8 -*-

import os


def getPackagePath(package):
    p = os.popen('rospack find ' + package)
    path = p.read()
    path = path[0:-1]
    return path
    

