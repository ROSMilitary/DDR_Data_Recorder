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




import os
from pathlib import Path
import queue
import shutil
import sys
import threading
import time

import rospy
import rospkg

from ddr_data_recorder.msg import Event
from std_msgs.msg import String

from tracker import SplitTracker

import msg_capture

ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')
SCRIPTS_DIR = os.path.join(PKG_DIR, 'scripts')

CONNECTIONS_PATH = os.path.join(SCRIPTS_DIR, 'connections')
sys.path.insert(1, CONNECTIONS_PATH)
from connections_generator import ConnectionsGenerator # pylint: disable=wrong-import-position, import-error

MARKDOWN_PATH = os.path.join(SCRIPTS_DIR, 'markdown')
sys.path.insert(1, MARKDOWN_PATH)
import markdown # pylint: disable=wrong-import-position, import-error


## @package ddr_data_recorder
# The Split Manager

## Manages the split tracker, yoinker, saver, and renamer
class SplitManager(): # pylint: disable=too-many-instance-attributes

    ## The constructory for SplitManager
    def __init__(self):

        rospy.init_node('split_manager', anonymous=False)

        ## @var pub
        # The publisher
        self.pub = rospy.Publisher('/ddr/event', Event, queue_size=10)

        self.ros_connections_pub = rospy.Publisher(
            '/ddr/ros_connections', String, queue_size=10)

        ## @var umc_timer
        # How long to sleep before replublishing an unexpectedModeChange as @n
        # a topicGroup change
        self.umc_timer = rospy.get_param("umcTimer")

        ## @var shadow_size
        # The duration of a shadow_record in seconds
        self.shadow_size = rospy.get_param("shadowSize")

        ## @var directory
        # The raw bag path input from the launch file
        self.directory = rospy.get_param("directory")

        ## @var bag_path
        # The full path where bags are saved
        self.bag_path = Path(self.directory).expanduser()

        ## @var gen_timer
        # How often to check for new bag creation
        self.gen_timer = rospy.get_param("genTimer")

        ## @var test_event
        # Split Yoinker (bag deletion) will be disabled when this value @n
        # returns true
        self.test_event = rospy.get_param("testEvent")

        ## @var tracker
        # An instance of the SplitTracker
        self.tracker = SplitTracker(self.bag_path, self.shadow_size,
                                    self.test_event)

        ## @var trigger
        # Prevents unexpectedModeChange from being republished as a @n
        # topicGroup change if another mode change has been detected within @n
        # the n second grace period
        self.trigger = False

        ## @var test_dir
        # The directory where bags outside of the sliding window are saved @n
        # when test_event == True
        if self.test_event:
            self.test_dir = self.bag_path.joinpath('test_event_bags')
            self.test_dir.mkdir(exist_ok=True)

        self.copier_lock = threading.Lock()
        self.copier_running = False

        self.generator_stop = False

        msg_capture.launch_message_capture(self.directory)


    ## Executed when a button press is detected
    # @param data The data received from the incoming message
    # @param event An instance of the ddr_data_recorder/Event message
    def _button_press(self, data, event):
        self.tracker.generator()
        if self.tracker.file_list:
            if data.eventType == "shadow record":
                event.eventID = "button press response"
                event.eventType = data.eventType
                event.dynamicRecord = False
                event.shadowRecord = False
                event.alertMsg = "Shadow record captured!"
                self.pub.publish(event)
                self.tracker.shadow_record()
            if data.eventType == "manual record":
                if self.tracker.active_bag:
                    event.eventID = "button press response"
                    event.eventType = data.eventType
                    event.dynamicRecord = False
                    event.shadowRecord = False
                    if self.tracker.manual_name:
                        event.alertMsg = "Manual record stopped!"
                    else:
                        event.alertMsg = "Manual record started!"
                    self.pub.publish(event)
                    # The name of the manual recording is received from the
                    # alert field and passed to the manual_record function
                    self.tracker.manual_record(data.alertMsg)
                else:
                    event.eventID = "button press response"
                    event.eventType = data.eventType
                    event.dynamicRecord = False
                    event.shadowRecord = False
                    event.alertMsg = "There are currently no active bag " \
                                        "recordings!"
                    self.pub.publish(event)
        else:
            event.eventID = "button press response"
            event.eventType = data.eventType
            event.dynamicRecord = False
            event.shadowRecord = False
            event.alertMsg = "There are no bag files in the ddr_bags " \
                                "directory!"
            self.pub.publish(event)


    ## Determines when to use SplitTracker
    # @param data The data received from the incoming message
    #
    # If an unexpectedModeChange is detected. This function will wait 15 seconds
    # and republish it as a normal mode change for dynamicRecording purposes.
    # If another mode change is detected during that time limit, the
    # unexpectedModeChange will not be republished.
    def split_tracking(self, data):
        event = Event()
        event.header.stamp = rospy.Time.now()

        if data.eventID == "button press":
            self._button_press(data, event)
        else:
            if data.eventID == 'topicGroup':
                self.trigger = True
            if data.shadowRecord:
                self.tracker.generator()
                self.tracker.shadow_record()
            if data.eventID == 'unexpectedModeChange':
                self.trigger = False
                time.sleep(self.umc_timer)
                if not self.trigger:
                    event.eventID = "topicGroup"
                    event.eventType = data.eventType
                    event.dynamicRecord = True
                    event.shadowRecord = False
                    event.alertMsg = ""
                    self.pub.publish(event)


    ## Updates file_key after files have been saved
    # @param file The file that belongs to the current event
    # @param event The current event
    def _update_file_key(self, file, key):
        if key in self.tracker.file_key[file][0]:
            self.tracker.file_key[file][0].remove(key)
            if self.tracker.file_key.opened:
                self.tracker.file_key.update_persistant()
        if key in self.tracker.file_key[file][1]:
            self.tracker.file_key[file][1].remove(key)
            if self.tracker.file_key.opened:
                self.tracker.file_key.update_persistant()


    @staticmethod
    ## Copies KML into the current recording directory
    # @param recording_directory The directory of the current recording
    def _copy_kml(recording_directory):
        kml_filename = rospy.get_param("topicXML")
        if kml_filename.startswith("/"):
            kml_filename = kml_filename[1:]
        kml_dir = os.path.join(
            SCRIPTS_DIR,
            'dynamic_recording',
            kml_filename
        )
        if not os.path.isfile(
                os.path.join(str(recording_directory), kml_filename)):

            shutil.copy2(kml_dir, str(recording_directory) + "/" + kml_filename)


    ## Saves bag files based on events in the event_key
    #
    # Saves files to a new folder named after the event. Removes the files from
    # from both event_key and file_key
    def split_saver(self): # pylint: disable=too-many-branches
        if self.tracker.event_key:
            # make a copy of the dictionary so we can remove items while
            # iterating over itself.
            cpy_key = self.tracker.event_key.copy()
            for key, files in cpy_key.items():
                og_dir = self.bag_path
                new_dir = Path("/", str(og_dir), key)
                new_dir.mkdir(exist_ok=True)
                self._copy_kml(new_dir)
                filescpy = files.copy()
                for file in filescpy:
                    try:
                        shutil.copy2(str(og_dir) + "/" + file,
                                     str(new_dir) + "/" + self.split_renamer(
                                         key, file))
                    except FileNotFoundError:
                        pass
                    try:
                        if file in files:
                            files.remove(file)
                            self._update_file_key(file, key)

                    except KeyError:
                        event = self.tracker.event_key.get(key)
                        if event is not None and file in event:
                            event.remove(file)
                        if self.tracker.event_key.opened:
                            self.tracker.event_key.update_persistant()
                        if file in self.tracker.file_list:
                            self.tracker.file_list.remove(file)
                        if file in self.tracker.file_key.keys():
                            del self.tracker.file_key[file]

                if (not self.tracker.event_key[key] and
                        key not in self.tracker.shadow_wait and
                        key not in self.tracker.manual_wait and
                        key != self.tracker.manual_name):
                    thread_markdown = threading.Thread(
                        target=markdown.generate_markdown,
                        args=(str(og_dir) + "/" + key + "/",
                              key))
                    thread_msg_capture = threading.Thread(
                        target=msg_capture.copy_msg_to_capture_folder,
                        args=(self.directory, key))
                    thread_markdown.start()
                    thread_msg_capture.start()
                    del self.tracker.event_key[key]

                    msg = String()
                    msg.data = str(new_dir)
                    self.ros_connections_pub.publish(msg)


    ## Renames bag files to match their event
    # @param event The name of the event folder where the bags are stored
    # @param current_name The current name of the bag file
    # @return The new file name
    @staticmethod
    def split_renamer(event, current_name):
        temp_list = current_name.split("_")
        temp_list.insert(1, event)
        return "_".join(temp_list)


    ## Removes bags from file_list and file_key if they do not exist
    # @param bag The name of the bag file
    def _split_yoinker_error_handler(self, bag):
        if bag in self.tracker.file_list:
            self.tracker.file_list.remove(bag)
        if bag in self.tracker.file_key.keys():
            del self.tracker.file_key[bag]



    ## Starts live bag copy if test_event == True @n
    # will start a copy on each new bag as it finishes filtering into test event
    def test_event_copier(self):
        # for bag in self.tracker.file_list:
        try:
            with self.copier_lock:
                if self.copier_running:
                    return
                self.copier_running = True

            self._unguarded_test_event_copier()

        finally:
            with self.copier_lock:
                self.copier_running = False


    def _unguarded_test_event_copier(self):
        while not self.tracker.copy_queue.empty():
            try:
                bag = self.tracker.copy_queue.get(block=False)
            except queue.Empty:
                return

            print('number of elements in queue: {}'.format(
                self.tracker.copy_queue.qsize()))
            bag_info = self.tracker.file_key.get(bag)
            if bag_info:
                if bag_info[2]: #if test event then
                    if not self.test_dir.joinpath(bag).exists():
                        print("copying " + bag)
                        shutil.copy(
                            str(self.bag_path.joinpath(bag)),
                            str(self.test_dir)
                        )
            else:
                self._split_yoinker_error_handler(bag)


    ## Deletes bag files if test_event == False
    #  No longer moves bag files if test_event == True
    def split_yoinker(self): # pylint: disable=too-many-branches
        yoink_list = []
        file_index = -1
        duration = 0
        for filename in reversed(self.tracker.file_list):
            duration += self.tracker.file_key[filename][4]
            if duration >= self.shadow_size:
                file_index = self.tracker.file_list.index(filename)
                break
        print("Total duration of bags in %s: %d" % (self.bag_path, duration))
        if file_index >= 0:
            yoink_list = self.tracker.file_list[:file_index]
        for bag in yoink_list:
            try:
                if (not self.tracker.file_key[bag][0] and
                        not self.tracker.file_key[bag][1]):
                    if not self.tracker.file_key[bag][2]:
                        print('Deleting {}'.format(bag))
                        Path(str(self.bag_path) + "/" + bag).unlink()
                    else:
                        if self.test_dir.joinpath(bag).exists():
                            print('Deleting {}'.format(bag))
                            try:
                                Path(str(self.bag_path) + "/" + bag).unlink()
                            except FileNotFoundError:
                                print("File {} not found - skipping delete.".format(bag))
                    self.tracker.file_list.remove(bag)
                    del self.tracker.file_key[bag]
            except KeyError:
                self._split_yoinker_error_handler(bag)



    ## Calls listed functions every 15 seconds @n
    # Runs on a separate thread
    def save_caller(self):
        while True:
            start_time = time.time()
            self.test_event_copier()
            self.split_yoinker()
            self.split_saver()
            end_time = time.time()
            gen_duration = end_time - start_time
            if gen_duration < self.gen_timer:
                time.sleep(self.gen_timer - gen_duration)

    ## Calls generator every 15 seconds @n
    # Runs on a separate thread
    def gen_caller(self):
        while not self.generator_stop:
            self.tracker.generator()
            if self.generator_stop:
                break
            time.sleep(self.gen_timer)

    ## Ran on rospy.on_shutdown()
    def clean_exit(self):
        print("Saving active bag files")
        time.sleep(1)
        for key in self.tracker.event_key.keys():
            event_path = self.bag_path.joinpath(key)
            event_path.mkdir(exist_ok=True)
            markdown_name = '{}_report.md'.format(key)
            file = open(str(event_path.joinpath(markdown_name)), "w+")
            file.write("Markdown generation incomplete. " \
                        "Run markdown.py on the capture.")
            file.close()

        self.generator_stop = True
        self.tracker.generator()

        self.tracker.active_bag = ""
        self.tracker.shadow_wait = []
        self.tracker.manual_wait = []
        self.tracker.manual_name = ""

        self.split_saver()

        ## Copies remaining splits to the test_event_bags folder when DDR exits
        if self.test_event:
            self._unguarded_test_event_copier()

            connections_generator = ConnectionsGenerator(
                os.path.join(SCRIPTS_DIR, 'dynamic_recording'))
            connections_generator.update_capture_ros_connections(
                str(self.test_dir))

        if self.tracker.event_key.opened:
            self.tracker.event_key.close()
        if self.tracker.file_key.opened:
            self.tracker.file_key.close()


    ## The listener function
    #
    # Initializes the split_manager node @n
    # Subscribes to /ddr/event
    def listener(self):

        self.tracker.generator()
        thread_gen = threading.Thread(target=self.gen_caller)
        thread_gen.daemon = True
        thread_gen.start()
        thread_save = threading.Thread(target=self.save_caller)
        thread_save.daemon = True
        thread_save.start()

        rospy.Subscriber("ddr/event", Event, self.split_tracking)

        rospy.spin()

        rospy.on_shutdown(self.clean_exit)


def main():
    spm = SplitManager()
    spm.listener()


if __name__ == "__main__":
    main()
