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



import operator
import os
import queue
import re
import subprocess
import threading
import time

import rospy

from std_msgs.msg import String
from filetracker import FileTracker

## @package ddr_data_recorder
# The bag recording Split tracker

## Manages the file structures for deleting and saving bag files
class SplitTracker(): # pylint: disable=too-many-instance-attributes

    ## The constructor for split_manager
    # @param bag_path the folder where bags are saved
    # @param shadow_size the number of splits needed for a shadow record
    def __init__(self, bag_path, shadow_size, test_event):

        ## @var test_event
        # Split Yoinker (bag deletion) will be disabled when this value @n
        # returns true
        self.test_event = test_event

        ## @var bag_path
        # The full path where bags are saved
        self.bag_path = bag_path

        # create /home/username/ddr_bags directory on startup if it does not @n
        # exist
        self.bag_path.mkdir(exist_ok=True)

        ## @var shadow_size
        # Retrieve the duration of a shadow_record in seconds
        self.shadow_size = shadow_size

        ## @var shadow_count
        # Keeps track of how many shadows have been called
        self.shadow_count = self.init_count("shadow")

        ## @var manual_count
        # Keeps track of how many manual records have been called
        self.manual_count = self.init_count("manual")

        ## @var manual_name
        # The name of the current manual recording
        self.manual_name = ""

        ## @var event_key
        # Keeps track of all bag files to be saved with the format of: @n
        # {event: [bag1, bag2, bag3]}
        self.event_key = FileTracker(os.path.abspath(
            os.path.join(os.path.dirname(__file__), "eventKey.json")))

        ## @var file_key
        # Keeps track of all bag files in the directory with the format of: @n
        # {filename: [[shadowList], manual_record(string), @n
        # test_event(bool), file_count(int)]}
        self.file_key = FileTracker(os.path.abspath(
            os.path.join(os.path.dirname(__file__), "fileKey.json")))

        self.event_key.open()

        self.file_key.open()

        for key in self.file_key.copy().keys():
            if key not in self.bag_path.glob('DDR_MAINBAGS*'):
                del self.file_key[key]

        ## @var file_list
        # A list of all files in the directory in the order they were created
        temp_list = [(key, value[3]) for key, value in self.file_key.items()]
        list.sort(temp_list, key=operator.itemgetter(1))
        self.file_list = [key for key, _ in temp_list]

        self.copy_queue = queue.Queue()
        for file in reversed(self.file_list):
            self.copy_queue.put(file)

        ## @var file_count
        # The number of files in the bag directory
        self.file_count = 0

        ## @var active_bag
        # Keep track of which bag files are currently active for saving purposes
        self.active_bag = ""

        ## @var manual_wait
        # Manual recordings that have been terminated are placed in this list @n
        # to ensure the next split gets saved to this recording
        self.manual_wait = []

        ## @var shadow_wait
        # Shadow recordings that have been triggered are placed in this list @n
        # to ensure the next split gets saved to this recording
        self.shadow_wait = []

        ## @var pub
        # The publisher
        self.pub = rospy.Publisher('/ddr/error', String, queue_size=10)

        ## @var generator_lock
        #  Locks generator for cuncurrent calls
        self.generator_lock = threading.Lock()


    def __del__(self):
        if self.event_key.opened:
            self.event_key.close()
        if self.file_key.opened:
            self.file_key.close()


    ## Determines if prior shadow and manual folders exist. @n
    # @return number of folders with folder_name
    def init_count(self, folder_name):
        count_list = [0]
        test = r'^(?P<name>' + folder_name + r')(_*)(?P<number>\d*)$'
        pattern = re.compile(test, re.IGNORECASE)
        for folder in self.bag_path.iterdir():
            if folder.is_dir():
                match = pattern.match(folder.name)
                if match:
                    if ''.join(match.groups()) == folder.name:
                        if match.group('number'):
                            count_list.append(int(match.group('number')))
                        else:
                            count_list.append(1)
        count_list.sort()
        return count_list[-1]


    ## Updates the file_key and event_key with a new shadow record
    def shadow_record(self):
        self.shadow_count += 1
        file_index = 0
        duration = 0
        shadow_name = "shadow" + str(self.shadow_count)
        for filename in reversed(self.file_list):
            duration += self.file_key[filename][4]
            if duration >= self.shadow_size:
                file_index = self.file_list.index(filename)
                break
        self.event_key.update({shadow_name : self.file_list[file_index:]})
        self.shadow_wait.append(shadow_name)
        for bag in self.file_list[file_index:]:
            self.file_key[bag][0].append(shadow_name)
            if self.file_key.opened:
                self.file_key.update_persistant()
        print(self.event_key)


    ## Updates the file_key and event_key with a new manual record
    # @param input_name The name of the manual recording
    def manual_record(self, input_name):
        if not self.manual_name:
            if input_name == "manual":
                self.manual_count += 1
                self.manual_name = input_name + str(self.manual_count)
            else:
                name_count = self.init_count(input_name)
                name_count += 1
                if name_count == 1:
                    self.manual_name = input_name
                else:
                    self.manual_name = input_name + "_" + str(name_count)
            self.event_key.update({self.manual_name: []})
        else:
            self.manual_wait.append(self.manual_name)
            self.manual_name = ""


    ## Uses the name of the bag file to determine what order they were @n
    # created in. Sorts the file_list based on this data
    def bag_sorter(self):
        bag_list = {}
        for bag in self.file_list:
            # grab the number before the first underscore
            num_name = int(bag.split("_")[0])
            if num_name not in bag_list:
                bag_list[num_name] = []
            # creates a dict w/ format {10: [10_idle_0.bag, 10_idle_1.bag...]}
            bag_list[num_name].append(bag)
        for key, bags in bag_list.items():
            sort_tool = []
            for bag in bags:
                name = bag[:-4] #strips .bag from filename
                if name.split("_")[-1].isdigit():
                    bag_num = name.split("_")[-1]
                else:
                    bag_num = 0
                sort_tool.append(int(bag_num))
            bag_list[key] = [x for _, x in sorted(zip(sort_tool, bags))]
        self.file_list = []
        for key in sorted(bag_list.keys()):
            self.file_list.extend(bag_list[key])


    ## Returns the duration of a given file within the bag_path
    # @param filename The name of the bag file
    # @return The duration of the provided bag
    def get_duration(self, filename):
        proc = subprocess.Popen("rosbag info " + str(self.bag_path) +
                                "/" + filename +
                                " | grep duration | awk '{ print $2 }' |" \
                                " sed 's/[(s)]//g'",
                                shell=True, stdout=subprocess.PIPE)
        output = proc.stdout.read()
        try:
            duration = float(output)
        except ValueError:
            duration = 0
            errmsg = "Unable to retreive the duration of " \
                "{}. Try running rosbag reindex".format(filename)
            self.pub.publish(errmsg)
        return duration


    ## Updates file_key and event_key if there is no current manual_record
    # @param filename The name of the bag file
    # @param duration The duration of the bag, filename
    def _gen_update_keys(self, filename, duration):
        self.file_key.update({filename : [self.shadow_wait, self.manual_wait,
                                          self.test_event, self.file_count,
                                          duration]})
        for event in self.manual_wait:
            self.event_key[event].append(filename)
            if self.event_key.opened:
                self.event_key.update_persistant()
        self.manual_wait = []


    ## Updates file_key and event_key if there is a current manual_record
    # @param filename The name of the bag file
    # @param duration The duration of the bag, filename
    def _gen_update_keys_manual(self, filename, duration):
        if self.manual_name not in self.event_key:
            self.event_key.update({self.manual_name: [filename]})
        else:
            self.event_key[self.manual_name].append(filename)
            if self.event_key.opened:
                self.event_key.update_persistant()
        self.file_key.update({filename : [self.shadow_wait, [self.manual_name],
                                          self.test_event, self.file_count,
                                          duration]})


    ## Updates event_key if there are items in the shadow_wait list
    # @param filename The name of the bag file
    def _gen_update_keys_shadow(self, filename):
        for event in self.shadow_wait:
            if event not in self.event_key:
                self.event_key.update({event : [filename]})
            else:
                self.event_key[event].append(filename)
                if self.event_key.opened:
                    self.event_key.update_persistant()


    ## Generates and updates the file_key and file_list based on new bags @n
    # in the bag directory
    def generator(self):
        start_time = time.time()
        print("generator started")
        temp_count = 0
        for file in self.bag_path.iterdir():
            if file.is_file() and ".bag" in file.name:
                filename = file.name
                with self.generator_lock:
                    if ("DDR_MAINBAGS" not in filename and
                            filename not in self.file_key):
                        duration = self.get_duration(filename)
                        self.file_count += 1
                        temp_count += 1
                        # If manual is false, it updates just the file_key
                        # If manual is true, it updates the event_key and the
                        # file_key and adds the manual event to the
                        # manual list in the file_key
                        if not self.manual_name:
                            self._gen_update_keys(filename, duration)
                        else:
                            self._gen_update_keys_manual(filename, duration)

                        self._gen_update_keys_shadow(filename)

                        self.shadow_wait = []
                        self.file_list.append(filename)
                        self.copy_queue.put(filename)

                    # Lets manager know that there is currently an active bag
                    # recording. Used strictly for error handling
                    else:
                        self.active_bag = filename


        # Sorts the list of bag files if more than one bag file was created
        # since the last time generator was ran or if this is the first time
        # running ddr
        if temp_count > 1:
            self.bag_sorter()
            for bag in self.file_list:
                self.file_key[bag][3] = self.file_list.index(bag)
            if self.file_key.opened:
                self.file_key.update_persistant()

        end_time = time.time()
        print(self.event_key)
        print("generator finished. Time: " + str(end_time - start_time))


def main():
    rospy.init_node('split_tracker', anonymous=False)
    spt = SplitTracker(rospy.get_param('directory'), 5, False)
    spt.generator()


if __name__ == "__main__":
    main()
