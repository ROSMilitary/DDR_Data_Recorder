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
from collections import deque
import threading
import datetime
import time
import rospy
import rosbag
from rosbag.bag import ROSBagException
from std_msgs.msg import String

## Documentation for RosparamCapture class
#
#  Class collects all rosparams at startup, and all changes to the
#  rosparams during runtime. When a capture occurs, it determines
#  which rosparams were active during the capture and writes all the
#  rosparams, theier start and end times and their values to a file
#  in the capture folder.
class RosparamCapture():
    ## The constructor.
    def __init__(self):
        rospy.init_node("rosparam_capture")
        ## @var self.captures_queue
        #  a member variable - a queue of ddr capture paths
        self.captures_queue = deque()
        ## @var self.capture_start_time
        #  a member variable - the start time of the capture being
        #  processed
        self.capture_start_time = 0.0
        ## @var self.capture_end_time
        #  a member variable - the end time of the capture being
        #  processed
        self.capture_end_time = 0.0
        ## @var self.capture_path
        #  a member variable - the path to the current capture being
        #  processed
        self.capture_path = ""
        ## @var self.param_names
        #  a member variable - a list of rosparams on the parameter server
        self.param_names = rospy.get_param_names()
        ## @var self.param_dict
        #  a member variable - master dictionary of rosparams, values and times.
        self.param_dict = {}
        ## @var self.param_server_dict
        #  a member variable - dictionary of rosparams and values received
        #  from the parameter server. This holds the latest values and will
        #  get updated into self.param_dict
        self.param_server_dict = {}
        print("start param setup!")
        time.sleep(5)
        print("sleep1!")
        self.get_dict_from_param_server()
        time.sleep(10)
        print("sleep2!")
        self.initialize_dictionary()

        time.sleep(5)
        print("sleep3!")

        thread1 = threading.Thread(target=self.insert_param_vals)
        thread1.start()
        time.sleep(5)
        print("sleep4")


        thread2 = threading.Thread(target=self.process_queue)
        thread2.start()
        print("rosparam setup complete! safe to create captures")
        rospy.Subscriber("/ddr/ros_connections", String,
                         self.ros_connections_callback)

    ## Documentation for the flatten_dict function
    #
    #  Recursive function that flattens a nested dictionary with a '/'
    #  between levels
    def flatten_dict(self, dic, separator='/', prefix=""):
        return {prefix + separator + k if prefix else k : v
                for kk, vv in dic.items()
                for k, v in self.flatten_dict(vv, separator, kk).items()
                } if isinstance(dic, dict) else {prefix : dic}

    ## Documentation for initialize_dictionary method
    #
    #  Method initializes the self.param_dict member variable using the list
    #  of all rosparams collected from the parameter server where each rosparam
    #  is a key in the dictionary and its values are the time when we start
    #  tracking rosparams and the current starting value of the rosparam.
    #  This method performs some paring of the data, stripping '/n' from
    #  rosparam values and deleting the rosparam /robot_model/robot_description
    #  from the dictionary of rosparams tracked.
    def initialize_dictionary(self):
        self.param_dict = dict.fromkeys(self.param_names)
        # Remove this ROSParam from dictionary and param_names list
        # This param value is a .xml file
        if "/robot_model/robot_description" in self.param_dict:
            del self.param_dict["/robot_model/robot_description"]
            self.param_names.remove("/robot_model/robot_description")
        unformatted_time = rospy.get_time()
        for param_name in self.param_names:
            try:
                param_val = self.flattened_dict[param_name]
            except KeyError:
                print("rosparam not found: ", param_name)
                print("if you see this you have a networking problem")
                print("I can't get your param values, restart roscore")
            if isinstance(param_val, str):
                if param_val.endswith('\n'):
                    param_val = param_val.strip('\n')
            val_pair = [unformatted_time, param_val]
            self.param_dict[param_name] = [val_pair]

    ## Documentation for get_dict_from_param_server method
    #
    #  Method pings the parameter server for all of the most recent rosparam
    #  values, which are returned in the form of a nested dictionary. After
    #  retrieving, the dictionary keys are formatted by first removing all
    #  '/' from the key names and then adding back a single '/' at the head
    #  of the name and then flattens the nested dictionary.
    def get_dict_from_param_server(self):
        self.param_server_dict = rospy.get_param('/')
        for param in self.param_server_dict:
            new_key = param.strip('/')
            new_key = '/' + new_key
            old_key = param
            self.param_server_dict[new_key] =\
                self.param_server_dict.pop(old_key)

        self.flattened_dict = self.flatten_dict(self.param_server_dict)

    ## Documentation for insert_param_vals method
    #
    #  The dictionary is designed to have a 2D list of values for each key with
    #  the following format:
    #        {'key': [[start_time, 'value'],
    #                 [end_time, 'value'],
    #                 [start_time, new_val],
    #                 [end_time, new_val]]}
    #
    #  Method continuously polls the parameter server looking for rosparam
    #  changes comparing the newly retrieved values to those stored in
    #  self.param_dict. If no new value is found, the end_time gets updated to
    #  the current time. If a new value is found, a new set of values is added
    #  to the dictionary.
    def insert_param_vals(self):
        # slowing down did not reduce cpu utilization
        #rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            self.get_dict_from_param_server()

            unformatted_time = rospy.get_time()
            format_time = datetime.datetime.utcfromtimestamp\
                    (unformatted_time).strftime('%H:%M:%S.2%f')
            for param_name in self.param_names:
                # Next two lines of code needs to be deleted. This is just for
                # debugging to show how quickly updates are occurring
                #if param_name == "/topicXML":
                    #print("TOPIC NAME: ", param_name, " Time: ", format_time)
                #try:
                param_val = self.flattened_dict[param_name]
                # except KeyError:
                #     print("2yikes this boi gone: ", param_name)
                #     print("better luck next time kiddo")
                #     print("if you see this you have a networking problem")
                #     print("I can't get your param values")

                if isinstance(param_val, str):
                    if param_val.endswith('\n'):
                        param_val = param_val.strip('\n')
                val_pair = [unformatted_time, param_val]

                vals = self.param_dict.get(param_name)
                num_rows = len(vals)

                if num_rows < 2:
                    #add a new list row to the key
                    self.param_dict[param_name].append(val_pair)

                elif num_rows >= 2:
                    try:
                        begin_param = vals[-2][1]
                        end_param = param_val

                        if begin_param == end_param:
                            #update the end time to the current time
                            vals[-1][0] = unformatted_time
                            self.param_dict.update(param_name=vals)
                        else:
                            self.param_dict[param_name].append(val_pair)
                    except KeyError:
                        print(param_name + " not found")
            #rate.sleep()

    ## Documentation for ros_connection_callback method
    #
    #  Method is the callback subscribed to the /ddr/ros_connections topic.
    #  Every time /ddr/ros_connections publishes a new capture has occurred,
    #  a capture path is added to a queue for processing
    def ros_connections_callback(self, msg):
        capture_path = str(msg.data)
        #print("CALLBACK FOR " + capture_path)
        self.captures_queue.append(capture_path)

    ## Documentation for process_queue method
    #
    #  Method to initiate processing of all captures in the queue. Method calls
    #  self.collect_capture_begin_end_times of the capture
    def process_queue(self):
        while not rospy.is_shutdown():
            if len(self.captures_queue) > 0:
                try:
                    self.capture_path = self.captures_queue.pop()
                    self.collect_capture_begin_end_times(self.capture_path)
                except IndexError:
                    print("Queue index out of range: ",
                          str(len(self.captures_queue)) + " " +
                          self.capture_path)
                    continue

    ## Documentation for collect_capture_begin_end_times method
    #
    #  Method takes the path of the current capture being worked on and collects
    #  the start and end times of the capture from the bag files in the capture
    #  folder and sets the member variables self.capture_start_time and
    #  self.Capture_end_time. After collecting the capture start and end times,
    #  the method calls the method query_and_write()
    def collect_capture_begin_end_times(self, capture_path):
        for bag_file in os.listdir(capture_path):
            if bag_file.endswith('.bag'):
                bag = rosbag.Bag(os.path.join(capture_path, bag_file))
                try:
                    start_time = bag.get_start_time()
                    end_time = bag.get_end_time()
                    if start_time == 0.0:
                        continue
                    if self.capture_start_time == 0.0 or \
                       start_time < self.capture_start_time:
                        self.capture_start_time = start_time
                    if end_time > self.capture_end_time:
                        self.capture_end_time = end_time
                except ROSBagException:
                    print(bag_file + " is empty")
        self.query_and_write()
        self.capture_start_time = 0.0
        self.capture_end_time = 0.0

    ## Documentation for query_and_write method
    #
    #  Method creates and opens a new file in the capture folder. It gets the
    #  self.param_dict key's values and queries these values to determine the
    #  active rosparam value(s) during the capture based on the capture start
    #  and end times. It then writes the dictionary keys, values and start and
    #  end times for each rosparam value which was active during the capture to
    #  a table in the file.
    #  This method calls the write_header method, which writes the capture
    #  start and stop times to the file and sets up the table.
    #  This method calls the write_values method, which writes the rosparam
    #  start, end, param name and value(s) to the markdown table.
    #  Note, this table is best viewed using a markdown reader.
    def query_and_write(self):
        capture_name = os.path.basename(self.capture_path)
        fout = open(self.capture_path + "/" + capture_name +\
            "_rosparam_report.md", 'a')
        self.write_header(fout, capture_name)
        for param in self.param_names:
            dict_vals = self.param_dict.get(param)
            for i in range(0, len(dict_vals), 2):
                param_start_time = datetime.datetime.utcfromtimestamp\
                    (dict_vals[i][0]).strftime('%H:%M:%S.2%f')
                param_end_time = datetime.datetime.utcfromtimestamp\
                    (dict_vals[i+1][0]).strftime('%H:%M:%S.2%f')
                if dict_vals[i][0] >= self.capture_start_time and\
                    dict_vals[i+1][0] <= self.capture_end_time:
                    self.write_values(fout, param_start_time, param_end_time,
                                      param, str(dict_vals[i][1]))
                elif dict_vals[i][0] <= self.capture_start_time and\
                    dict_vals[i+1][0] >= self.capture_start_time:
                    self.write_values(fout, param_start_time, param_end_time,
                                      param, str(dict_vals[i][1]))
                elif dict_vals[i][0] <= self.capture_end_time and\
                    dict_vals[i+1][0] >= self.capture_end_time:
                    self.write_values(fout, param_start_time, param_end_time,
                                      param, str(dict_vals[i][1]))
                else:
                    print("dict_vals: " + param_start_time + " " +
                          param_end_time + param + str(dict_vals[i][1]) +
                          " are outside the capture window")
        print("ROSParam report for " + capture_name + " has been generated")
        fout.close()

    ## Documentation for write_header method
    #
    #  Method writes to the output file, the capture name, the start and end
    #  times of the capture and sets up the markdown table.
    #  This method is called by the query_and_write method
    def write_header(self, fout, capture_name):
        format_cap_start_time = datetime.datetime.utcfromtimestamp\
                    (self.capture_start_time).strftime('%b %d, %Y %H:%M:%S.%f')
        format_cap_end_time = datetime.datetime.utcfromtimestamp\
                    (self.capture_end_time).strftime('%b %d, %Y %H:%M:%S.%f')
        fout.write("## Capture Name:   " + capture_name + "\n")
        fout.write("\n")
        fout.write("#### Capture Start: " + format_cap_start_time + '\n')
        fout.write("#### Capture End: " + format_cap_end_time + '\n')
        fout.write("#### Rosparams\n")
        fout.write("| Start Time | End Time | ROSParam | Value |\n")
        fout.write("| --- | --- | --- | --- |\n")

    ## Documentation for the write_values method
    #
    #  Method takes the output file, the rosparam start time, end time, the
    #  param and it's value and writes to the output file.
    #  This method is called by the query_and_write method.
    @classmethod
    def write_values(cls, fout, start_time, end_time, ros_param, value):
        fout.write("|" + start_time + "|" + end_time + "|" +
                   ros_param + "|" + value + "|\n")

def main():
    RosparamCapture()
    rospy.spin()

## Driver Script
if __name__ == "__main__":
    main()
