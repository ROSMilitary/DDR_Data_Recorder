#!/usr/bin/env python3

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
import datetime
import hashlib
import re
import mmap
import sys
from operator import itemgetter
import rospy
import rosbag
from gps_module import gps_information
from gear_mode_module import gear_information
from topic_group_module import topic_group_information
SYS_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(1, SYS_PATH)
import util # pylint: disable=C0413

## Rosbag Report generator functionally reads all the bags
# from a given directory and generates a human readable synopsis
#  of the bags in a markdown file.
class RosbagReport(): # pylint: disable=R0902, R0904
    _FILENAME_SUFFIX = '_report.md'

    def __init__(self, directory, captureName):
        # @var generates all user given class Objects
        self.user_input_call_objects()

        ## Duration Objects
        # @var Start Time of Bags
        self.total_start_time = None

        # @var End Time of Bags
        self.total_end_time = None

        # @var Duration of all the bags in the recording folder
        self.duration = 0.0

        # @var is all the topics in given bags
        self.topics = set()

        # @var a list of bags that will be used for timeline operations,
        # if the user would like to see the order of something their output
        # can be placed in the user_input_timeline func
        self.timeline_bag_data = []

        # @var Prefix of the file
        self.file_prefix = captureName

        # @var the private Final Output
        self._internal_report = None

        # A setter for our directory
        self._set_directory(directory)

        # @var a list of bag paths within the given capture directory
        # that will be traversed over
        # Gathers all of the bag files
        self.bags = self.gather_bag_files()

        ## @var reindex
        # A list of bag files that may need to be reindexed
        self.reindexable_bags = []
        self.previous_hash = None
        self.did_hashes_match = ""
        self.prior_hashes = self.check_for_prior_hashes()


    ## Gets the path
    @property
    def path(self):
        if not getattr(self, '_path', None):
            path = os.path.join(self.directory, '{}{}'.format( \
                self.file_prefix, RosbagReport._FILENAME_SUFFIX))
        return path
    @property
    ## Gets the directory
    def directory(self):
        return self._directory
    ## Sets the directory based on a given path
    def _set_directory(self, value):
        if value.endswith('/'):
            self._directory = os.path.dirname(value)
        else:
            self._directory = value
    ## Python Action to open the report on enter
    def __enter__(self):
        self.open_report()
        return self

    ## Python Action to close the report on exit
    def __exit__(self, type, value, traceback): # pylint: disable=W0622
        self.close_report()
    ## Initalizes the report
    @property
    def _reporter(self):
        try:
            self._internal_report
        except:
            raise RuntimeError('Markdown file not created yet!')
    ## Setter for the report
    @_reporter.setter
    def _reporter(self, value):
        self._internal_report = value
    ## Opens the report
    def open_report(self):
        if self._internal_report:
            raise RuntimeError('Markdown file already open. \
                        Close this before creating another one')
        self._internal_report = open(self.path, mode='w')
    ## Action to close the report
    def close_report(self):
        if not self._internal_report:
            raise RuntimeError('Markdown file not created yet')
        self._internal_report.close()
    ## Reorganizes Rosbags from the directory to a singular list
    def gather_bag_files(self):
        # Error checking on input.
        if self.directory is None:
            return None
        return util.get_files_list(str(self.directory)) # pylint: disable=no-member
    ## Tries to pull the kml ros parameter
    def pull_ros_param(self): # pylint: disable=R0201
        try:
            return rospy.get_param('topicXML')
        except:# rospy.ROSException:
            return "kml.xml"
    ## Gets information about the rosbag
    def get_rosbag_info(self, bag): # pylint: disable=R0201
        try:
            return str(bag).split("\n")
        except RuntimeError:
            print("There was a problem getting the \
                    rosbag information for the bag.")
    ## Gets the basename of a file
    def get_basename_of(self, filename): # pylint: disable=R0201
        return os.path.basename(filename)

    ## The input for the Rosbag Report all information
    # iterativly goes through this function.
    # Calls userInput to check each bag individually
    def read_bags(self):
        for bag_path in self.bags:
            try:
                with rosbag.Bag(bag_path, 'r') as bag:
                    if bag:
                        rosbag_info = self.get_rosbag_info(bag)
                        try:
                            self.get_duration_start_end_time(rosbag_info)
                        except RuntimeError:
                            self.reindexable_bags.append( \
                                self.get_basename_of(bag.filename))
                        self.read_topics(bag)
                        self.user_input_read_each_bag(bag)
                    else:
                        self.reindexable_bags.append(
                            self.get_basename_of(bag.filename))
            except:
                pass


    def get_total_size(self):
        total_size = 0
        for bag_path in self.bags:
            try:
                with rosbag.Bag(bag_path, 'r') as bag:
                    total_size += bag.size
            except:
                pass
        return total_size


    def read_topics(self, bag):
        for topic in bag.get_type_and_topic_info()[1].keys():
            self.topics.add(topic)

    def write(self, data):
        return self._internal_report.write('{}\n  '.format(data))

    def write_report(self):
        try:
            self.write_base()
        except RuntimeError:
            print("Bags contain no data.")
    ## Writes the base of the markdown Report.
    # all bags should have these parameters at the minimum
    # a hash is generated to protect validity of the Report
    def write_base(self):
        ## Default Output
        self.write("# Capture Name: {}".format(self.get_basename_of(\
            self.file_prefix)))
        self.write("## File Names:")
        self.write("  \n  ".join(self.get_basename_of(x) for x in \
            util.get_sorted_files_list(self.directory))) # pylint: disable=E1101
        if self.reindexable_bags:
            self.write("\n## Files that may need to be reindexed:")
            self.write("\n  ".join(bag for bag in self.reindexable_bags))
        total_size = self.get_total_size()
        self.write("\n## Total Size:       {} {}".format(round( \
            total_size/1000000, 1), "MB"))
        self.write("## Number of Splits: {}".format(len(self.bags)))
        self.write("## Total Space Saved Using DDR: {}".format( \
            util.bytes_to_human_str(util.size_saving(self.directory)))) # pylint: disable=E1101
        self.write("## Hash of Used kml file (md5): {}".format( \
            "".join(self.hash_manager())))

        ## Time Output
        self.write("\n## Date: {}".format(str( \
            self.total_start_time).split()[0]))
        self.write("## Start Time:   {}".format(str( \
            self.total_start_time).split()[1]))
        self.write("## End Time:     {}".format(str( \
            self.total_end_time).split()[1]))
        self.write("## Total Duration (s): {}{}".format( \
            round(self.duration, 2), "s"))
        user_outputs = self.user_input_report_additions()
        _ = [self.write(data) for data in user_outputs]


        ## Timeline functionallity
        self.write("\n## Timeline (UTC):\n")
        self.timeline_bag_data.extend(self.topic_group.timeline_bag_data)
        self.timeline_bag_data.extend(self.gears.timeline_bag_data)
        self.timeline_bag_data.sort(key=itemgetter(2))
        for topic, msg, timestamp in self.timeline_bag_data:
            formatted_time = datetime.datetime.utcfromtimestamp( \
                timestamp.to_time()).strftime('%b %d, %Y %H:%M:%S.%f')
            formatted_msg = self.user_input_timeline(topic, msg)
            if formatted_msg != '':
                self.write("{0}\t{1}\n  ".format(formatted_time, formatted_msg))

        self.write("\n## Topics:\n")
        self.topics = list(self.topics)
        self.topics.sort()
        self.write("  \n  ".join(self.topics))


    ## Generates hashes from all locations that a topic based XML file exists
    def hash_manager(self):
        list_of_hashes = []
        list_of_hashes += self.get_install_dir_hash( \
            self.get_kml_dir_path(), self.pull_ros_param())
        list_of_hashes += self.prior_hashes
        self.prior_hashes.clear()
        return list_of_hashes

    ## Walks the directory in question to find .xml and .md
    # files that were used previously
    def check_for_prior_hashes(self):
        list_of_hashes = []
        for root, _, files in os.walk(os.path.dirname(self.directory)):
            if self.file_prefix in str(root):
                for file in files:
                    if file.endswith(".xml"):
                        list_of_hashes +=  \
                            self.generate_capture_response(root, file)
                    if file.endswith(".md"):
                        list_of_hashes += \
                            self.open_previous_markdown(root, file)
        list_of_hashes += self.did_hashes_match
        return list_of_hashes
    ## Takes the input directory and adds a newly generated hash with label
    def generate_capture_response(self, root, file):
        label = "\n### kml Hash from current capture directory {}"
        capture_location = os.path.join(root, file)
        if capture_location:
            self.match_previous_hashes(self.generate_hash(\
                capture_location), location="capture")
            return label.format("found.")
        else:
            return label.format("not found.")
    ## Takes the newly found markdown and searches for a hash to return
    def open_previous_markdown(self, root, file): # pylint: disable=R0201
        hash_of_previous_markdown = ""
        label = "\n### kml Hash from previous markdown {}"
        with open(os.path.join(root, file), 'rb', 0) as \
                _file, \
                mmap.mmap(_file.fileno(), \
                0, \
                access=mmap.ACCESS_READ) \
                as previous_markdown:
            str_of_markdown = str(previous_markdown[1:])
            search_result = re.search( \
                r"\b([a-f\d]{32}|[A-F\d]{32})\b", \
                str_of_markdown)
            if search_result:
                hash_of_previous_markdown = label.format("found.")
                self.match_previous_hashes(search_result, location="md")
            else:
                hash_of_previous_markdown = label.format("not found.")
            previous_markdown.close()
        return hash_of_previous_markdown

    ## Getter for the kml directory
    def get_kml_dir_path(self): # pylint: disable=R0201
        return os.path.dirname(os.path.realpath(__file__))
    ## Finds the hash of the kml currently in the capture directory
    def get_install_dir_hash(self, kml_dir_path, topic_xml):
        return "{}".format( \
            self.generate_hash(kml_dir_path + \
            '/../dynamic_recording/' + topic_xml)
            )
    ## Generates a MD5 hash
    def generate_hash(self, path): # pylint: disable=R0201
        with open(path) as kml_file:
            buffer = kml_file.read()
            hash_of_kml = hashlib.md5()
            hash_of_kml.update(buffer.encode('utf-8'))
            return hash_of_kml.hexdigest()
    ## Matches hashes between previous MD and capture KMLs
    def match_previous_hashes(self, new_hash, location):
        if self.previous_hash is None:
            self.previous_hash = [new_hash, location]
        else:
            if new_hash == self.previous_hash[0] and \
                location != self.previous_hash[-1]:
                self.did_hashes_match = "\n### KML Hash match"
            else:
                self.did_hashes_match = "\n### KML Hash doesn't match"
    ## Manages duration, start, and end time in the bags
    def get_duration_start_end_time(self, rosbag_info):
        for line in rosbag_info:
            duration, start, end = self.search_for_duration_start_end(line)
            if duration:
                self.duration += self.add_to_duration(duration.group("value"))
            elif start:
                self.set_start_time(start.group("value"))
            elif end:
                self.set_end_time(end.group("value"))
    ## Searches for Duration, Start, and End time in each bag
    def search_for_duration_start_end(self, line): # pylint: disable=R0201
        duration = re.search(r"duration:\s+(?P<value>.*)$", line)
        start = re.search(r"start:.*\((?P<value>[\.\d]+)\)$", line)
        end = re.search(r"end:.*\((?P<value>[\.\d]+)\)$", line)
        return duration, start, end
    ## Sets start time based on if it's the earliest bag in the capture
    def set_start_time(self, start_time):
        start = datetime.datetime.fromtimestamp(float(start_time))
        if self.total_start_time is None or start < self.total_start_time:
            self.total_start_time = start
    ## Sets end time based on if it's the latest bag in the capture
    def set_end_time(self, end_time):
        end = datetime.datetime.fromtimestamp(float(end_time))
        if  self.total_end_time is None or end > self.total_end_time:
            self.total_end_time = end
    ## Continuiously adds to the duration value
    def add_to_duration(self, duration): # pylint: disable=R0201
        return float(duration.replace("s", ""))

# Call Class Functions here:
################################################################
    def user_input_call_objects(self):
        self.gps = gps_information()
        self.gears = gear_information()
        self.topic_group = topic_group_information()

    def user_input_read_each_bag(self, bag):
        self.gps.read_gps(bag)
        self.gears.gear_interpreter(bag)
        self.topic_group.topic_group_interpreter(bag)

    def user_input_timeline(self, topic, msg):
        formatted_str = ''
        if topic == gear_information.GEAR_TOPIC:
            formatted_str = self.gears.format_output(topic, msg)
        elif topic == topic_group_information.TOPIC_GROUP_TOPIC:
            formatted_str = self.topic_group.format_output(topic, msg)
        return formatted_str


    def user_input_report_additions(self):
        formatted_additions = []

        formatted_additions.append(self.gps.write_gps())

        return formatted_additions

################################################################
################################################################
