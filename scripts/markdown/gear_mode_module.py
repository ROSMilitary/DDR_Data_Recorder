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


import rospy


class gear_information():
    GEAR_TOPIC = rospy.get_param("gearTopic")
    def __init__(self):
        self.gear_bags = []
        self.timeline_bag_data = []
        self.formattedMsg = ''

    ## Converts an index into the text representation of a gear
    # @param gearNum int index of the gear
    # @return str representation of the gear
    def get_gear_name(self, gearNum):
        GEARS = ['Park', 'Reverse', 'Neutral', 'Drive', 'Drive2']
        if gearNum < 0 or gearNum >= len(GEARS):
            return 'Unknown'
        return GEARS[gearNum]

    ## Takes the topic and message from the bag and returns a formatted message string based on the topic
    # @param topic str representing the topic the message is from
    # @param msg temporary subclass of genpy.message.Message representing the message for the specified topic
    # @return Formatted string for timeline section of markdown file
    def format_output(self, topic, msg):
        # Format msg to show current gear and desired gear
        output_str = ''
        if topic == gear_information.GEAR_TOPIC:
            output_str = 'Current Gear: {}; Desired Gear: {}'.format(self.get_gear_name(msg.gear), self.get_gear_name(msg.desired_gear))
        return output_str

    ## Gear Change items
    def gear_interpreter(self, bag):

        self.gear_bags = list(bag.read_messages(topics=[gear_information.GEAR_TOPIC]))
        filteredGearBags = []
        for previous_element, element in zip(self.gear_bags, self.gear_bags[1:]):
            if (element[1].gear != previous_element[1].gear) or (element[1].desired_gear != previous_element[1].desired_gear):
                filteredGearBags.append(element)
        self.timeline_bag_data.extend(filteredGearBags)
