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


import os
import sys
import xml.etree.ElementTree as ET

SYS_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'dynamic_recording'))
sys.path.insert(1, SYS_PATH)

from TopicParser import TopicParser # pylint: disable=import-error, wrong-import-position


## Help check topics in a kml file
#  Use TopicParser to parse a kml file and compare the values in the kml
#  for two modes to determine the overlap
class TopicBagChecker:

    ## The constructor
    #  @param path The path to the kml file
    def __init__(self, path):
        self.path = path
        self.topic_parser = TopicParser(self.path)
        self.modes = []
        self.topic_set = {}
        self.build_modes()
        self.build_modes_set()


    ## Get all of the modes from the kml
    def build_modes(self):
        root = ET.parse(self.path).getroot()
        modes = root.findall('groupList')[0].findall('topicGroup')[0]
        self.modes = [mode.tag for mode in  modes]


    ## Get all of the topics for each mode in the kml
    def build_modes_set(self):
        for mode in self.modes:
            self.topic_parser.conditions["topicGroup"] = mode
            self.topic_parser.processGroupsNode()
            self.topic_parser.finalizeTopics()
            self.topic_set[mode] = set(self.topic_parser.topics["final"])


    ## Do the processing for a bag file
    #  Compare the topics between the previous mode and the current mode
    #  to determine the topics that were not in the previous mode but are
    #  in the current mode so they can be reset
    #  @param previous_mode The mode that the previous bag was recorded in
    #  @param current_mode The mode that the current bag was recorded in
    #  @returns A set containing the topics that are in the current bag
    #  but were not in the previous bag or the empty set
    def process_bag(self, previous_mode, current_mode):
        if current_mode in self.topic_set.keys() and \
           previous_mode in self.topic_set.keys():

            return self.topic_set[current_mode] - self.topic_set[previous_mode]

        return set()
