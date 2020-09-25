#!/usr/bin/env python3


#******************************************************************************
#
#"Distribution A: Approved for public release; distribution unlimited. OPSEC #4046"
#
#PROJECT: DDR
#
# PACKAGE         :
# ORIGINAL AUTHOR :
# MODIFIED DATE   :
# MODIFIED BY     :
# REVISION        :
#
# Copyright (c) 2020 DCS Corporation
#
# Unlimited Rights assigned to the U.S. Government
#
# This material may be reproduced by or for the U.S Government pursuant
# to the copyright license under the clause at DFARS 252.227-7013.  This
# notice must appear in all copies of this file and its derivatives.
#******************************************************************************
#
#Copyright (c) 2019-2020 U.S. Federal Government (in countries where recognized)
#Permission is hereby granted, free of charge, to any person obtaining a copy of
#this software and associated documentation files (the "Software"), to deal in
#the Software without restriction, including without limitation the rights to use,
#copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
#Software, and to permit persons to whom the Software is furnished to do so,
#subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
#DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#DEALINGS IN THE SOFTWARE.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import math
import rospy
from gps_common.msg import GPSFix
from geometry_msgs.msg import PoseStamped
from math import radians, sin, cos, sqrt, asin
class gps_information():

    def __init__(self):


        self.distance = 0
        self.ListSize = 0
        self.GPSlist = []
        self.origin = None
        self.messages = []
        self.numOfMsgs = 0
        self.gpsPerBag = []

    def write_gps(self):
        if self.ListSize <= 0:
            #TODO: Put missing topic name
            #formattedOutput = "## Final Distance:   {}\n".format(" No GPS data found!!")
            #formattedOutput.append("## Final Location:   ,{}".format(" No GPS data found!!"))
            formatted_final_dist = "## Final Distance:   {}\n".format(" No GPS data found!")
            formatted_final_location = "## Final Location:   {}".format(" No GPS data found!")
            return formatted_final_dist + formatted_final_location
        # self.handle_local_xy_origin(self.messages[0])
        # self.GPSlist.append(self.local_xy_to_wgs84( self.messages[0].x , self.messages[0].y))

        for interval in range(0,self.ListSize-2):
            # handle_local_xy_origin(self.messages[interval])
            # GPSlist.append(self.local_xy_to_wgs84( self.messages[interval+1].x , self.messages[interval+1].y))
            self.distance += abs(self.haversine(self.messages[interval].latitude , self.messages[interval].longitude, self.messages[interval+1].latitude, self.messages[interval+1].longitude))
        number_of_decimals = 3
        formatted_final_dist = "## Final Distance:   {}{}\n".format(round(self.distance, number_of_decimals), " Kilometers")
        formatted_final_location = "## Final Location:   {},{}".format(self.messages[-1].longitude,self.messages[-1].latitude)
        return formatted_final_dist + formatted_final_location

    def read_gps(self, bag):
        ## GPS
        self.gpsTopic = rospy.get_param("gpsTopic")
        self.gpsPerBag = bag.read_messages( topics=[self.gpsTopic])
        self.numOfMsgs = bag.get_message_count(topic_filters=[self.gpsTopic])
        for i in range(0, self.numOfMsgs):
            topic, msg, t = next(self.gpsPerBag)
            self.messages.append(msg)

        #TODO: better variable name
        self.ListSize = len(self.messages)

    def handle_local_xy_origin(self, origin):
        self.local_xy_origin = origin
        self.reference_latitude = self.local_xy_origin.pose.position.y * math.pi / 180
        self.reference_longitude = self.local_xy_origin.pose.position.x * math.pi / 180
        reference_heading = 0.0  # in radians
        self.cos_heading = math.cos(reference_heading)
        self.sin_heading = math.sin(reference_heading)

        earth_eccentricity = 0.08181919084261
        earth_equator_radius = 6378137.0

        depth = -self.local_xy_origin.pose.position.z
        p = earth_eccentricity * math.sin(self.reference_latitude)
        p = 1.0 - p * p
        rho_e = earth_equator_radius * (1.0 - earth_eccentricity * earth_eccentricity) / (math.sqrt(p) * p)
        rho_n = earth_equator_radius / math.sqrt(p)
        self.rho_lat = rho_e - depth
        self.rho_lon = (rho_n - depth) * math.cos(self.reference_latitude)



    ## Transforms a point in the WGS84 coordinate frame to the local_xy frame.
    # @param lon: Longitude
    # @param lat: Latitude
    # @return: An (x,y) coordinate pair in the local_xy frame
    def wgs84_to_local_xy(self, lon, lat):

        rlat = lat * math.pi / 180
        rlon = lon * math.pi / 180
        dlat = (rlat - self.reference_latitude) * self.rho_lat
        dlon = (rlon - self.reference_longitude) * self.rho_lon
        y = dlat * self.cos_heading + dlon * self.sin_heading
        x = -1.0 * (dlat * self.sin_heading - dlon * self.cos_heading)
        return x, y


    ## Transforms a point in the local_xy frame to the WGS84 coordinate frame.
    # @param x: x-position in local_xy
    # @param y: y-position in local_xy
    # @return: An (lon,lat) coordinate pair in the local_xy frame
    def local_xy_to_wgs84(self, x, y):

        dlon = self.cos_heading * x - self.sin_heading * y
        dlat = self.sin_heading * x + self.cos_heading * y
        rlat = (dlat / self.rho_lat) + self.reference_latitude
        rlon = (dlon / self.rho_lon) + self.reference_longitude

        lat = rlat * 180 / math.pi
        lon = rlon * 180 / math.pi

        return lon, lat


    ## The Haversine formula
    # The standard Mathmatical formula for taking two longitudes and latitudes points and
    # projecting them onto the earths surface to find the exact distance.
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6372.8  # Earth radius in kilometers
        dLat = radians(lat2 - lat1)
        dLon = radians(lon2 - lon1)
        lat1 = radians(lat1)
        lat2 = radians(lat2)
        a = sin(dLat / 2)**2 + cos(lat1) * cos(lat2) * sin(dLon / 2)**2
        c = 2 * asin(sqrt(a))
        return R * c #returns Kilometers
