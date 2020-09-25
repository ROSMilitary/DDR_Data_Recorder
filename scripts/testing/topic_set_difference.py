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
import rosbag

SYS_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'dynamic_recording'))
sys.path.insert(1, SYS_PATH)

from TopicParser import TopicParser # pylint: disable=E0401,C0413


## @package Data DirectoR
## @author
## @date 1/10/2020
## @class TopicSetDifference
# This class is used as a way of verifying that KML, DynamicRecord, and
# TopicParser are working correctly.
# Its constructor needs path to a KML.
# Call 'process_bag()' with a bag file passed as the parameter.
# Currently, it outputs text to the console to determine the validity of the
# bag when compared to the states in the kml.
# A possible improvement could be comparing the initial modes against one
# another, telling the user which modes are already subsets or supersets of
# the other ones.
class TopicSetDifference:
    ## ctor
    # @param kml_file_path str The path to the KML.
    def __init__(self, kml_file_path: str = None) -> None:
        self.kml_file_path = kml_file_path
        self.topic_parser = TopicParser(self.kml_file_path)
        self.modes = []
        self.topic_set = {}
        self.build_modes()
        self.build_modes_set()


    ## Builds up the different modes from the KML.
    def build_modes(self) -> None:
        root = ET.parse(self.kml_file_path).getroot()
        modes = root.findall('groupList')[0].findall('topicGroup')[0]
        self.modes = [mode.tag for mode in  modes]


    ## Build topic_set for all the modes
    def build_modes_set(self) -> None:
        for mode in self.modes:
            self.topic_parser.conditions["topicGroup"] = mode
            self.topic_parser.processGroupsNode()
            self.topic_parser.finalizeTopics()
            self.topic_set[mode] = set(self.topic_parser.topics["final"])


    ## Process the bag
    # @param bag rosbag.Bag The rosbag to process.
    def process_bag(self, bag: rosbag.Bag) -> None:
        info = bag.get_type_and_topic_info()
        bag_topics = set(info.topics.keys())

        print('Bag Name: {}'.format(bag.filename))
        size = len(max(self.topic_set.keys(), key=len)) + 1
        for mode, topics in self.topic_set.items():
            if bag_topics.issubset(topics):
                print(('{:<' + str(size) + '} --SUBSET by {} topics').format(
                    mode + ':', len(topics - bag_topics), size=size))
            elif bag_topics.issuperset(topics):
                print(('{:<' + str(size) + '} ++SUPERSET').format(
                    mode + ':', size=size))
            else:
                print(('{:<' + str(size) + '} off by {}').format(
                    mode + ':', len(topics - bag_topics), size=size))
        print('-------------------')


## Driver code
def main() -> None:
    print('Ctrl+C to quit or enter empty string for bag name.')
    tsd = TopicSetDifference('../dynamic_recording/kml.xml')

    # We've been given a list of bag names, process them.
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            print(arg)
            bag = rosbag.Bag(arg)
            tsd.process_bag(bag)
        return

    # We're going to be given bag names as user input.
    try:
        while True:
            bag_path = input('Enter Path to Bag: ')

            if bag_path:
                bag = rosbag.Bag(bag_path)
                tsd.process_bag(bag)

            # Exit on empty string.
            else:
                break

    except KeyboardInterrupt:
        print('\nExiting...')


if __name__ == '__main__':
    main()
