#!/usr/bin/python

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

import json
import rospy
import time
from std_msgs.msg import String

## @package ddr_data_recorder
# The event detection feature of DDR

## Subcribes to multiple topics in order to detect multiple types of events
#
# Publishes to /ddr/api as the events occur
class EventDetector():

    ## The contructor for EventDetector
    def __init__(self):
        rospy.init_node('mode_state_bridge', anonymous=False)
        ## @var pub
        # The publisher for the /ddr/event topic
        self.pub = rospy.Publisher('/ddr/api', String, queue_size=10)

    def kickstart(self):
        kickstart_waypoint = {}
        # write your eventID as it appears in the KML
        kickstart_waypoint['eventID'] = "topicGroup"
        # if you want to change the topics to record, this should be set to True
        kickstart_waypoint['dynamicRecord'] = True
        # if you want to trigger a shadow record, this should be set to True
        # if you just want a shadow record, eventID and eventType can be left empty
        kickstart_waypoint['shadowRecord'] = False
        # this is your subcategory as it appears in the KML
        kickstart_waypoint['eventType'] = "kickstart"
        # alert message to push out to
        kickstart_waypoint['alertMsg'] = "Waiting for autonomy mode change. Kickstarted to default topic group."
        self.pub.publish(json.dumps(kickstart_waypoint))
        print("Kickstart Successful!")

    ## Listens to your chosen topic and publishes to /ddr/api when an event is detected
    # @param data the topic data
    def callback(self,data):
        event = {}
        #this may also need if elif else logic if you have multiple eventIDs
        event['eventID'] = "topicGroup"
        event['dynamicRecord'] = True
        event['shadowRecord'] = False
        event['eventType'] = "this"
        event['alertMsg'] = "that"

        #place your if conditionals to decide your alert message and eventType
#        if your_trigger == desired_condition:
#            event['eventType'] = "this"
#            event['alertMsg'] = "that"
#        elif your_trigger == desired_condition2:
#            event['eventType'] = "this2"
#            event['alertMsg'] = "that2"

        self.pub.publish(json.dumps(event))


    ## The listener function
    # Initialized the node @n
    # Subscribes to your topic of interest
    def listener(self):
        mode = rospy.get_param("modeTopic")
        rospy.Subscriber(mode, '''YOUR ROS MESSAGE TYPE''', self.callback)
        time.sleep(5)
        self.kickstart()
        rospy.spin()


if __name__=='__main__':
    ed = EventDetector()
    ed.listener()
