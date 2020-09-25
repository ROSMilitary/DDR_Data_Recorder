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



## @package LiveMileageCounter
#
#  Author:
#  Date:   August 2019
#
#  LiveMileageCounter contains a single class which subscribes to
#       your GPS topic set in launch file with rosparam gpsTopic
#       outputting a GPSFix message /ddr/mileage_counter_msg
#
#  It collects details from the gps topic to calculate the total distance
#  traveled and the speed of the vehicle.
#
#  The /ddr/mileage_counter_msg topic published by gui.py in the gui package
#  publishes options from the gui with start, stop and reset buttons.
#  Functionality for stop, start and reset and handled within the Mileage
#  class. This class then publishes the distance traveled and the current
#  speed of the vehicle.

from math import radians, sin, cos, sqrt, asin #For GPS distace triangulation
import rospy
from gps_common.msg import GPSFix
from std_msgs.msg import String

## Documentation for Mileage class.
#
#  The Mileage class subscribes to
#           gpsTopic --> rosparam for your gps topic that outputs GPSFix
#            longitude and latitude waypoints and vehicle speed

#       /ddr/mileage_counter_msg - user options from gui.py in gui package
#       start, stop and reset mileage counter options
#  Mileage publishes the current vehicle speed and calculates and publishes the
#  distance traveled.
class Mileage():
    ## The constructor.
    def __init__(self):
        rospy.init_node("mileage_counter")
        # @var used to grab the gps topic for calculating mileage
        self.gpsTopic = rospy.get_param("gpsTopic")

        rospy.Subscriber(self.gpsTopic, GPSFix, self.radio_data_cb)
        rospy.Subscriber("/ddr/mileage_counter_msg", String,
                         self.mileage_options_cb)
        ## @var self.pub
        #  a member variable - publisher for /ddr/mileage_counter topic
        self.pub = rospy.Publisher("/ddr/mileage_counter", String,
                                   queue_size=10)
        ## @var self.pub1 - publisher for /ddr/vehicle_speed topic
        #  a member variable
        self.pub1 = rospy.Publisher("/ddr/vehicle_speed", String, queue_size=10)
        #self.pub2 = rospy.Publisher("/ddr/error", String, queue_size=10)

        ## @var self.vehicle_speed
        #  a member variable - current speed of the vehicle
        self.vehicle_speed = 0.0
        ## @var self.distance
        #  a member variable - distance traveled since start record distance
        #  option initialized
        self.distance = 0.0
        ## @var self.latitude1
        #  a member variable - 2nd most recent latitude waypoint
        self.latitude1 = 0.0
        ## @var self.latitude2
        #  a member variable - most recent latitude waypoint
        self.latitude2 = 0.0
        ## @var self.longitude1
        #  a member variable - 2nd most recent longitude waypoint
        self.longitude1 = 0.0
        ## @var self.longitude2
        #  a member variable - most recent longitude waypoint
        self.longitude2 = 0.0
        ## @var self.radius
        # Earth's radius in kilometers
        self.radius = 6372.8
        ## @var self.distance_initialized
        #  a member variable - used as a flag to state whether distance has been
        #  initialized. If not initialized, initialize with two sets of the same
        #  longitude and latitude points. Note that two sets of points are
        #  needed to calculate distance. At the beginning, distance is zero.
        self.distance_initialized = False
        ## @var self.start_counter
        #  a member variable - flag stating whether mileage counter has been
        #  requested
        self.start_counter = True
        ## @var self.stop.counter
        #  a member variable - flag stating if stop counter has been requested
        self.stop_counter = False
        ## @var self.reset_counter
        #  a member variable - flag stating if mileage reset has been requested
        self.reset_counter = False

    ## Documentation for get_distance() method.
    #  @param self The object pointer.
    #  get_distance() takes two parameters, the current latitude and longitude
    #  location and calculates the distance the vehicle moved since the last
    #  checkpoint, adds this distance cumulatively and then publishes a
    #  formatted cumulative distance on the topic /ddr/mileage_counter
    #  This method also publishes the current vehicle speed on the topic
    #  /ddr/vehicle_speed
    def get_distance(self, latitude, longitude):
        distance_msg = String()

        if self.start_counter:
            self.reset_counter = False
            self.stop_counter = False
            if self.distance_initialized:
                self.latitude1 = self.latitude2
                self.longitude1 = self.longitude2
                self.latitude2 = latitude
                self.longitude2 = longitude
            else:
                self.latitude1 = latitude
                self.longitude1 = longitude
                self.latitude2 = latitude
                self.longitude2 = longitude
                self.distance_initialized = True
            self.distance += self.haversine(self.latitude1, self.longitude1,
                                            self.latitude2, self.longitude2)
        elif self.reset_counter:
            self.distance_initialized = False
            self.start_counter = False
            self.stop_counter = False

        elif self.stop_counter:
            self.start_counter = False
            self.reset_counter = False

        distance = format(round(self.distance, 2), '.2f')
        distance_msg.data = str('{:0>10}'.format(distance))
        self.pub.publish(distance_msg)

    ## Documentation for radio_data_cb() method.
    #  @param self The object pointer.
    #  Callback method to receive longitude and latitude data points and current
    #  vehicle speed from self.gpsTopic topic and sets the class instance
    #  variable, self.vehicle_speed and calls the
    #  git_distance(latitude,longitude) method which calculates and increments
    #  the class instance variable self.distance
    def radio_data_cb(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        self.vehicle_speed = msg.speed
        speed_msg = String()
        # convert from m/s to km/hr
        self.vehicle_speed = self.vehicle_speed * 3600 / 1000
        speed = format(round(self.vehicle_speed, 1), '.1f')
        speed_msg.data = str('{:0>5}'.format(speed))
        self.pub1.publish(speed_msg)
        #self.pub2.publish(str(msg.speed))
        self.get_distance(latitude, longitude)

    ## Documentation for mileage_options_cb() method.
    #  @param self The object pointer.
    #  callback method to receive user option to start, stop or reset the
    #  displayed mileage shown in gui.py in the gui package and sets the
    #  following class instance variables
    #       self.start_counter, self.reset_counter, self.stop_counter
    def mileage_options_cb(self, msg):
        if msg.data == "start":
            self.start_counter = True
            self.reset_counter = False
            self.stop_counter = False
        elif msg.data == "stop":
            self.distance_initialized = False
            self.start_counter = False
            self.stop_counter = True
            self.reset_counter = False
        elif msg.data == "reset":
            self.distance_initialized = False
            self.start_counter = False
            self.stop_counter = False
            self.reset_counter = True
            self.distance = 0.0
            self.get_distance(0.0, 0.0)

    ## Documentation for haversine() function
    #
    #  The standard Mathmatical formula to calculate the great-circle
    #  distance between two points - that is, the shortest distance over
    #  the earth's surface - giving an 'as-the-crow-flies' distance between
    #  the points (ignoring any hills they fly over)
    def haversine(self, lat1, lon1, lat2, lon2):
        d_lat = radians(lat2 - lat1)
        d_lon = radians(lon2 - lon1)

        lat1 = radians(lat1)
        lat2 = radians(lat2)

        square_of_half_of_chord_length = (sin(d_lat / 2)**2 + cos(lat1) *
                                          cos(lat2) * sin(d_lon / 2)**2)
        angular_distance_radians = (2 *
                                    asin(sqrt(square_of_half_of_chord_length)))

        return self.radius * angular_distance_radians #returns Kilometers

## Documentation for the main() method
#
def main():
    mil = Mileage()
    rospy.spin()

## Driver Script
if __name__ == "__main__":
    main()
