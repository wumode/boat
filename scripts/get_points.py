# -*- coding:utf-8 -*-
# -*- author:wumo -*-

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline
import utm
import time
import re
import xlwt
from xlrd import *
from xlutils.copy import copy

if __name__ == '__main__':
    point_file = open("points.txt", "r")
    xml_points = open("map.xml", "w+")
    # print(type(s))
    pattern = re.compile(r'(\d+.?\d*),(\d+.?\d*)')
    s_s = point_file.readlines()
    lat_list = []
    lon_list = []
    gps_lat_list = []
    gps_lon_list = []
    gps_list = []
    key_list = [18, 41]
    for s in s_s:
        matchObj = pattern.match(s)
        if matchObj:
            # print(float(matchObj.group(2)))
            # print(matchObj.groups())
            gps_list.append((float(matchObj.group(1)), float(matchObj.group(2))))
            # utm_pos = utm.from_latlon(float(matchObj.group(1)), float(matchObj.group(2)))
            # print(utm_pos)

            # print(utm_pos[0], utm_pos[1])
            # lon_list.append(utm_pos[1])
            # lat_list.append(utm_pos[0])

    xml_points.write("<gps>\n")
    for i in key_list:
        str_p = """     <point>
        <latitude>%f</latitude>
        <longitude>%f</longitude>
    </point>
""" % (gps_list[i][0], gps_list[i][1])
        xml_points.write(str_p)
    xml_points.write("</gps>")
    xml_points.close()