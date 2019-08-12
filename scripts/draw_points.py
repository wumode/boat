# -*- coding:utf-8 -*-
# -*- author:wumo -*-

import matplotlib.pyplot as plt
import numpy as np
from flask import Flask, request, render_template, redirect
import time
import urllib
import time
import re
import os
import utm
import hashlib
from urllib import parse
import requests

AK = "YYY0vwUqhSqAIVNy8xvVuqRqVg4G71Aq"
SK = "R8svft34jVZIxYVkEN79WinOi8Lc3RVg"


def WGS84_TO_BD09(gps84):
    param = {"coords": "%f,%f" % (gps84[1], gps84[0]), "ak": AK, "from": 1, "to": 5, "output": "json"}
    url_request = "/geoconv/v1/?coords=%s&ak=%s&from=%d&to=%d&output=%s" % (param["coords"],
        param["ak"], param["from"], param["to"], param["output"])
    encodedStr = parse.quote(url_request, safe="/:=&?#+!$,;'@()*[]")
    rawStr = encodedStr + SK
    SN = hashlib.md5(parse.quote_plus(rawStr).encode("utf-8")).hexdigest()
    param["sn"] = SN
    url_final = "http://api.map.baidu.com" + encodedStr + "&sn=%s" % SN
    r = requests.get(url_final)
    print(r.json())
    bd09 = []
    if r.json()['status'] == 0:
        bd09.append(r.json()['result'][0]['y'])
        bd09.append(r.json()['result'][0]['x'])
    else:
        print("err status: %d" % r.json()['status'])
    return bd09


app = Flask(__name__)


@app.route('/', methods=['GET'])
def MapShow():
    url_unquote_path = 'index.html'
    return render_template('index.html')


if __name__ == '__main__':
    # print(end-start)
    # a = utm.from_latlon(32.35972, 125.68415)
    # print(a)
    # b = utm.to_latlon(a[0], a[1], 50, 'S')
    # print(b)
    # WGS84_TO_BD09([37.0, 122.0])
    # exit(0)
    point_file = open("points.txt", "r")
    map_js = open("static/map.js", "w+")
    # print(type(s))
    pattern = re.compile(r'(\d+.?\d*),(\d+.?\d*)')
    s_s = point_file.readlines()
    lat_list = []
    lon_list = []
    gps_lat_list = []
    gps_lon_list = []
    gps_list = []

    for s in s_s:
        matchObj = pattern.match(s)
        if matchObj:
            # print(float(matchObj.group(2)))
            # print(matchObj.groups())
            gps_list.append((float(matchObj.group(1)), float(matchObj.group(2))))
            utm_pos = utm.from_latlon(float(matchObj.group(1)), float(matchObj.group(2)))
            print(utm_pos)

            # print(utm_pos[0], utm_pos[1])
            lon_list.append(utm_pos[1])
            lat_list.append(utm_pos[0])

    local_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    map_js.write("var data = [\n")
    for i in range(len(gps_list)):
        map_js.write("  {name: '%d', value: 30},\n" % i)
    map_js.write("];\nvar geoCoordMap = {\n")
    i = 0
    bd09_list = []
    for gps_pos in gps_list:
        bd09 = WGS84_TO_BD09(gps_pos)
        map_js.write("  '%d':[%f,%f],\n" % (i, bd09[1], bd09[0]))
        bd09_list.append(bd09)
        i += 1
    map_js.write("};\n")
    map_js.write("""
    
var convertData = function (data) {
    var res = [];
    for (var i = 0; i < data.length; i++) {
        var geoCoord = geoCoordMap[data[i].name];
        if (geoCoord) {
            res.push({
                name: data[i].name,
                value: geoCoord.concat(data[i].value)
            });
        }
    }
    return res;
};

    option = {
    title: {
        text: 'GPS Points',
        subtext: 'Data are recorded in %s',
        sublink: 'http://www.westsite.cn:8901',
        left: 'center',
        textStyle: {
            color: '#fff'
        }
    },
    tooltip : {
        trigger: 'item'
    },
    bmap: {
        center: [%f, %f],
        zoom: 19,
        roam: true,
        mapStyle: {
            styleJson: [
                    {
                        "featureType": "water",
                        "elementType": "all",
                        "stylers": {
                            "color": "#044161"
                        }
                    },
                    {
                        "featureType": "land",
                        "elementType": "all",
                        "stylers": {
                            "color": "#004981"
                        }
                    },
                    {
                        "featureType": "boundary",
                        "elementType": "geometry",
                        "stylers": {
                            "color": "#064f85"
                        }
                    },
                    {
                        "featureType": "railway",
                        "elementType": "all",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "highway",
                        "elementType": "geometry",
                        "stylers": {
                            "color": "#004981"
                        }
                    },
                    {
                        "featureType": "highway",
                        "elementType": "geometry.fill",
                        "stylers": {
                            "color": "#005b96",
                            "lightness": 1
                        }
                    },
                    {
                        "featureType": "highway",
                        "elementType": "labels",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "arterial",
                        "elementType": "geometry",
                        "stylers": {
                            "color": "#004981"
                        }
                    },
                    {
                        "featureType": "arterial",
                        "elementType": "geometry.fill",
                        "stylers": {
                            "color": "#00508b"
                        }
                    },
                    {
                        "featureType": "poi",
                        "elementType": "all",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "green",
                        "elementType": "all",
                        "stylers": {
                            "color": "#056197",
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "subway",
                        "elementType": "all",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "manmade",
                        "elementType": "all",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "local",
                        "elementType": "all",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "arterial",
                        "elementType": "labels",
                        "stylers": {
                            "visibility": "off"
                        }
                    },
                    {
                        "featureType": "boundary",
                        "elementType": "geometry.fill",
                        "stylers": {
                            "color": "#029fd4"
                        }
                    },
                    {
                        "featureType": "building",
                        "elementType": "all",
                        "stylers": {
                            "color": "#1a5787"
                        }
                    },
                    {
                        "featureType": "label",
                        "elementType": "all",
                        "stylers": {
                            "visibility": "off"
                        }
                    }
            ]
        }
    },
    series : [
        {
            name: 'Points',
            type: 'scatter',
            coordinateSystem: 'bmap',
            data: convertData(data),
            symbolSize: function (val) {
                return val[2] / 10;
            },
            label: {
                normal: {
                    formatter: '{b}',
                    position: 'right',
                    show: false
                },
                emphasis: {
                    show: true
                }
            },
            itemStyle: {
                normal: {
                    color: '#ddb926'
                }
            }
        },
        {
            name: 'Points',
            type: 'effectScatter',
            coordinateSystem: 'bmap',
            data: convertData(data),
            symbolSize: function (val) {
                return val[2] / 10;
            },
            showEffectOn: 'render',
            rippleEffect: {
                brushType: 'stroke'
            },
            hoverAnimation: true,
            label: {
                normal: {
                    formatter: '{b}',
                    position: 'right',
                    show: false
                }
            },
            itemStyle: {
                normal: {
                    color: '#ddb926',
                    shadowBlur: 10,
                    shadowColor: '#333'
                }
            },
            zlevel: 1
        }
    ]
};
    """ % (local_time, bd09_list[0][1], bd09_list[0][0]))
    map_js.close()
    lat = np.asanyarray(lat_list)
    lon = np.asanyarray(lon_list)
    print(gps_list[0])
    #exit(0)
    a = min(lat)

    xy_max = max(int(max(lat)), int(max(lon)))
    xy_min = min(int(min(lat)), int(min(lon)))
    xy_min -= 2
    xy_max += 2
    x = range(xy_min, xy_max, 10)
    y = range(xy_min, xy_max, 10)

    # plt.xlim(xy_min, xy_max)
    # plt.ylim(xy_min, xy_max)
    # plt.figure(figsize=(6, 6))
    # plt.plot(lat_list, lon_list, 'green')
    # plt.scatter(lat_list, lon_list, marker='o')
    # plt.xlabel('x')
    # plt.ylabel('y')
    #
    # plt.grid(True)
    # plt.axis('tight')
    # plt.show()
    app.run(host='127.0.0.1', port=5000)
