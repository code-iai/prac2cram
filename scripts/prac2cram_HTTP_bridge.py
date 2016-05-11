#!/usr/bin/env python

import urllib
import urllib2
import rospy

from prac2cram.srv import *

def getParamsMap(paramNames, paramValues):
    paramsMap = {}
    numNames = len(paramNames)
    numValues = len(paramValues)
    if numNames > numValues:
        numNames = numValues
    for k in range(numNames):
        paramsMap[paramNames[k]] = paramValues[k]
    return paramsMap

def onPOSTRequest(req):
    url = req.url
    paramsPOST = urllib.urlencode(getParamsMap(req.parameter_names, req.parameter_values))
    res = POSTRequestResponse()
    res.result = urllib2.urlopen(url, paramsPOST).read()
    return res

def http_bridge_node():
    rospy.init_node('http_bridge_node')
    s = rospy.Service('prac2cram_http_bridge', POSTRequest, onPOSTRequest)
    rospy.spin()

if __name__ == "__main__":
    http_bridge_node()

