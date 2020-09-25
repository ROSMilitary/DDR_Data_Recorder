#!/usr/bin/python3


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

import glob
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading

import rosbag
import rospkg
import rospy

from ddr_data_recorder.msg import Event
from std_msgs.msg import String

from TopicParser import TopicParser

ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')

SCRIPTS_PATH = os.path.join(PKG_DIR, 'scripts')
sys.path.insert(1, SCRIPTS_PATH)
import util # pylint: disable=wrong-import-position

CONNECTIONS_PATH = os.path.join(SCRIPTS_PATH, 'connections')
sys.path.insert(1, CONNECTIONS_PATH)
from connections_generator import ConnectionsGenerator # pylint: disable=wrong-import-position, import-error


## The Dynamic Recoridng functionallity
#  Used to dynamically record and kill bags depending on
#  various states changes and the topics within those states.
#  Uses a few ROS parameters such as split size, path, and
#  overlap to run this funciton must me made in the launch file.
class DynamicRecorder: # pylint: disable=too-many-instance-attributes
    ## The Constructor
    def __init__(self):
        # The path for the kml.xml file location
        xml_path = os.path.dirname(__file__) + rospy.get_param("topicXML")
        # A topic parser object used in the record and filter function
        self.topic_parser = TopicParser(xml_path)

        ## @var The message coming off the callback function used to determine
        #  the event state and what topics will be filtered into what bags
        self.data = None

        # @var used to change bag sizes
        self.split_size = rospy.get_param("splitSize")


        ## @var This list is going to track the modes that the current active
        #  bag has been active for. We are going to use this to get all the
        #  topics for all the modes in this list.
        self.active_bag_modes = []

        # Temporary, is set on callback
        self.bag_name = 'DeezBags'

        ## Thread Locks
        # @var used to change running status
        self.run_lock = threading.Lock()
        # @var used to send error messages
        # TODO: Is this used?
        self.thread_lock = threading.Lock()

        # @var bool Used to track if we're running or not.
        self.is_running = False
        self.is_filtering = False

        # @var used to change the bag directory location
        self.directory = rospy.get_param("directory")

        ## @var bag_path
        #  The full path where bags are saved
        self.bag_path = Path(self.directory).expanduser()

        # Create ddr_bags directory on startup if it does not exist
        self.bag_path.mkdir(exist_ok=True)

        test_event = rospy.get_param("testEvent")

        ## @var init_index
        #  used to initialize the index
        if test_event:
            self.test_event_dir = self.bag_path.joinpath('test_event_bags')
            self.test_event_dir.mkdir(exist_ok=True)
            ddr_bags_init = DynamicRecorder.get_mode_counter_index(
                self.bag_path)
            test_bags_init = DynamicRecorder.get_mode_counter_index(
                self.bag_path.joinpath("test_event_bags"))
            if ddr_bags_init < test_bags_init:
                self.init_index = test_bags_init
            else:
                self.init_index = ddr_bags_init
        else:
            self.init_index = DynamicRecorder.get_mode_counter_index(
                self.bag_path)

        # @var int the directory we're recording
        self.bag_process_counter = self.init_index

        # Check for existing DDR_MAINBAGS files
        split_number = 0
        mainbags_list = util.get_mainbags_files_list(self.bag_path) # pylint: disable=no-member
        if mainbags_list:
            self.init_index += 1
            util.sort_mainbags_files_list(mainbags_list) # pylint: disable=no-member
        for bag in mainbags_list:
            if bag.endswith(".active"):
                Path(bag).unlink()
            else:
                new_bag_name = "{}_floatingMainBag_{}.bag".format(
                    self.init_index, split_number)
                shutil.move(bag, str(self.bag_path.joinpath(new_bag_name)))
                split_number += 1

        ## @var is the value of the splits that increment on filters and
        #  resets of bag_process_counter
        self.bag_split_counter = 0

        # @var Checks the previous event ID for any changes in the process
        self.previous_event_id = None
        # @var Checks the previous event type for any changes in the process
        self.previous_event_type = None

        self.previous_event_topics = []
        self.has_mode_changed = False

        ## @var List of bags that are currently being filtered. Useful if we
        #  start filtering multiple bags at once.
        self.bags_currently_being_filtered = []

        self.ns_add = set()
        self.ns_subtract = set()

        self.connection_generator = ConnectionsGenerator(
            os.path.join(SCRIPTS_PATH, 'dynamic_recording'))

        self.update_ros_connections()

        ## Initializes the node to listen for events and
        # hand off messages to callback function
        self.ros_connections_sub = rospy.Subscriber(
            '/ddr/ros_connections', String, self.ros_connections_callback)
        self.event_sub = rospy.Subscriber(
            "/ddr/event", Event, self.recording_callback)
        self.pub = rospy.Publisher(
            "/ddr/bag_filter_status", String, queue_size=10)
        rospy.spin()

    ## Gets the index of the topic to actively
    #  @returns index int The exact index to start using for recording.
    @staticmethod
    def get_mode_counter_index(file_path):
        file_list = []
        index = 0
        for file in file_path.iterdir():
            if file.is_file() and \
               ".bag" in file.name and \
               "DDR_MAINBAGS" not in file.name:
                # 1_idle_3.bag
                file_list.append(int(file.name.split("_")[0]))
        if file_list:
            file_list.sort()
            index = file_list[-1]
        return index

    ## Used to safely start dynamic recording
    def start_recording(self):
        if not self.is_running:
            self.is_running = True
            self.record()

    ## Uses the TopicParser class to generate a list of topics
    #  Requires the event and file location to determine topics
    #  TODO Fill out this documenation
    #  @param event_id string
    #  @param event_type string
    #  @return topic_list string a space-separated string of all the topics
    def topics_to_filter(self, event_id=None, event_type=None):
        if event_id is None:
            event_id = self.data.eventID

        if event_type is None:
            event_type = self.data.eventType

        self.topic_parser.conditions[event_id] = str(event_type)
        self.topic_parser.processGroupsNode()
        self.topic_parser.finalizeTopics()
        topic_list = self.topic_parser.getFinalTopics()
        self.previous_event_topics = topic_list
        return topic_list


    ## Core functionallity of the program. The process is one singular stream of
    #  recording through the whole program. The topic parser is initialized with
    #  all of the condition modes in mind to record all topics. Takes the topics
    #  generated from the topicParser and then creates a rosbag record messages.
    #  Did not use rosbag API for runtime error purposes.
    def record(self):
        self.topic_parser.processGroupsNode()
        self.topic_parser.finalizeTopics()
        topics = self.topic_parser.getSuperset()
        self.ns_add = self.topic_parser.getnsAdd()
        # TODO: Is this used?
        self.ns_subtract = self.topic_parser.getnsSubtract()

        ## Starting bash command
        # Cannot use rosbag API because it does not let us choose settings
        dynamic_record_command = ["rosbag record "]
        dynamic_record_command.append(
            '--split --duration=' + str(self.split_size))
        dynamic_record_command.append(
            "-O " + str(self.bag_path) + "/DDR_MAINBAGS")
        dynamic_record_command.append(
            " ".join(topics))
        dynamic_record_command.append(
            " -e \"(" + " | ".join(self.ns_add) + ")\" ")
        # dynamic_record_command.append(
        #     " -x \"(" + " | ".join(self.ns_subtract) + ")\" ")
        dynamic_record_command.append("__name:=DDR_MAIN_RECORDING")
        dynamic_record_command.append(">> /dev/null")

        ## Call Command on Console
        try:
            subprocess.Popen(" ".join(dynamic_record_command), shell=True)
        except subprocess.CalledProcessError:
            print("There was a problem getting the rosbag recording started.")

    ## Used to safely start the filtering thread
    def start_filtering(self):
        if not self.is_filtering:
            thread = threading.Thread(target=self.find_bag_to_filter)
            thread.daemon = True
            thread.start()

    ## Finds the bag to filter based on the name of the bag in the file location
    #  Calls the filter function within it's own seperate thread
    def find_bag_to_filter(self):
        while self.is_running:
            for file in self.bag_path.iterdir():
                if file.is_file() and \
                   file.name.endswith(".bag") and \
                   "DDR_MAINBAGS" in file.name and \
                   "filtering" not in file.name and \
                   file.name not in self.bags_currently_being_filtered:

                    self.bags_currently_being_filtered.append(file.name)
                    self.filter_bag(file.name)

    ## Filters a specific bag file depending on the modes inside of
    #  self.active_bag_modes.
    #  @param bag_name_to_filter string The name of the bag file to filter.
    def filter_bag(self, bag_name_to_filter):
        # Grabs the last mode the bag split was active for.
        new_bag_last_mode = self.active_bag_modes[-1]

        post_filter_topic_list = self.topics_to_filter(
            event_id="topicGroup", event_type=" ".join(self.active_bag_modes))

        # Add "filtering" to the bag name while it is being filtered
        temp_list = bag_name_to_filter.split("_")
        temp_list.insert(2, "filtering")
        new_bag_name_with_path = str(self.bag_path) + '/' + '_'.join(temp_list)
        bag_path_to_filter = str(self.bag_path) + '/' +  bag_name_to_filter
        final_bag_path = str(self.bag_path) + '/' + \
                         str(self.bag_process_counter) + "_" + \
                         new_bag_last_mode + "_" + \
                         str(self.bag_split_counter) + '.bag'

        with rosbag.Bag(bag_path_to_filter, 'r') as bag:
            connection_info = bag.get_type_and_topic_info()
            self.connection_generator.add_connection(
                final_bag_path, connection_info)

        input_arguments_to_filter = []
        input_arguments_to_filter.append(os.path.join(
            SCRIPTS_PATH,
            'dynamic_recording/bag_filter'
        ))
        input_arguments_to_filter.append(bag_path_to_filter)
        input_arguments_to_filter.append(new_bag_name_with_path)
        input_arguments_to_filter.append(post_filter_topic_list)

        try:
            proc = subprocess.Popen(" ".join(input_arguments_to_filter),
                                    shell=True, stdout=subprocess.PIPE)
            output = proc.stdout.read().decode("utf-8")
            # Rename filtered bag to final name
            shutil.move(new_bag_name_with_path, final_bag_path)
            if output == "1":
                if bag_name_to_filter in self.bags_currently_being_filtered:
                    self.bags_currently_being_filtered.remove(
                        bag_name_to_filter)
                else:
                    print("Problem removing a bag from list. Bag Name: {} " \
                          "List of Bags: {}".format(
                              bag_name_to_filter,
                              self.bags_currently_being_filtered))
                # No matter what, we want to keep the last mode we were in
                # for the next filtering of a bag file.
                self.active_bag_modes = [self.active_bag_modes[-1]]

                with self.run_lock:
                    print("we're incrementing the bag")
                    self.bag_split_counter += 1
            else:
                msg = String()
                msg.data = "Error output of bag_filter: " + str(output)
                self.pub.publish(msg)
        except subprocess.CalledProcessError:
            print("There was a problem getting the rosbag recording started.")

    ## Called on shutdown. Filters the active bag in the directory.
    def filter_active_bag(self):
        for file in self.bag_path.iterdir():
            if file.is_file() and \
               file.name.endswith(".bag.active") and \
               "DDR_MAINBAGS" in file.name:

                self.bags_currently_being_filtered.append(file.name)
                self.filter_bag(file.name)

    ## Takes in a String message and starts a new bag while getting rid of the
    #  previous one it is done this way to create a desired overlap
    #  @param data ddr_data_recorder/Event.msg - The message we're handling
    def recording_callback(self, data):

        if data.dynamicRecord:
            self.start_recording()
            self.data = data

            #Checks the previous event and mode to change record processes
            if data.eventID != self.previous_event_id or \
               data.eventType != self.previous_event_type:
                self.active_bag_modes.append(data.eventType)
                print("active_bag_modes: ", self.active_bag_modes)

                index = DynamicRecorder.get_mode_counter_index(self.bag_path)
                if index > self.init_index:
                    self.bag_process_counter = index + 1
                else:
                    self.bag_process_counter = self.init_index + 1
                print("self.bag_process_counter {}".format(
                    self.bag_process_counter))
                self.bag_split_counter = 0
                self.bag_name = data.eventType
                print("self.bag_name:" + self.bag_name + ":")
                self.previous_event_id = data.eventID
                self.previous_event_type = data.eventType
                self.has_mode_changed = True

            self.start_filtering()


    def update_ros_connections(self):
        capture_folders = glob.glob(
            '{}/*/'.format(os.path.expanduser(self.directory)))
        for capture_folder in capture_folders:
            self.connection_generator.update_capture_ros_connections(
                os.path.abspath(capture_folder))

        self.connection_generator.clean_up_persistant(
            os.path.expanduser(self.directory))


    def ros_connections_callback(self, msg):
        self.connection_generator.update_capture_ros_connections(msg.data)


if __name__ == "__main__":
    rospy.init_node("data_director_event")

    try:
        DYNAMIC_RECORDER = DynamicRecorder()
        rospy.on_shutdown(DYNAMIC_RECORDER.filter_active_bag)

    except rospy.ROSInterruptException:
        pass
