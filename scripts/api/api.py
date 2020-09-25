#!/usr/bin/python3

#******************************************************************************
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
from ddr_data_recorder.msg import Event
from std_msgs.msg import String



## @package ddr_data_recorder
# The event detection feature of DDR

## Subscribes to /ddr/api in order to detect multiple types of events
# Publishes to /ddr/event as the events occur
class API():

    ## The contructor for API
    def __init__(self):

        ## @var pub
        # The publisher for the /ddr/event topic
        self.pub = rospy.Publisher('/ddr/event', Event, queue_size=10)


    ## Listens to the /ddr/api topic and publishes to /ddr/event when an event is detected
    # @param data the /ddr/api topic data
    def callback(self, msg):

        # An instance of the ddr_data_recorder/Event message type
        event = Event()

        data = json.loads(msg.data)
        event.eventID = data['eventID']
        event.eventType = data['eventType']
        event.dynamicRecord = data['dynamicRecord']
        event.shadowRecord = data['shadowRecord']
        event.alertMsg = data['alertMsg']

        self.pub.publish(event)


    ## The listener function
    # Initialized the ddr_api node @n
    def listener(self):
        rospy.init_node('ddr_api', anonymous=False)
        rospy.Subscriber("/ddr/api", String, self.callback)
        rospy.spin()


if __name__=='__main__':
    api = API()
    api.listener()
