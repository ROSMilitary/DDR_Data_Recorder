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
import json
import os
import re
import sys

import rospkg

ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')

SPLIT_MANAGER_PATH = os.path.join(PKG_DIR, 'scripts', 'SplitManager')
sys.path.insert(1, SPLIT_MANAGER_PATH)
from filetracker import FileTracker # pylint: disable=wrong-import-position, import-error


class ConnectionsGenerator():
    CONNECTIONS_FILENAME = 'connections.json'


    def __init__(self, path):
        self.path = path
        self._ros_connections = FileTracker(os.path.join(
            self.path, ConnectionsGenerator.CONNECTIONS_FILENAME))
        self._ros_connections.open()


    def __del__(self):
        if self._ros_connections.opened:
            self._ros_connections.close()


    @property
    def path(self):
        return self._path


    @path.setter
    def path(self, value):
        self._path = ConnectionsGenerator._clean_path(value)


    ## Takes a path to a directory and cleans it up for use
    #  @param path The path to clean up
    @staticmethod
    def _clean_path(path):
        path = os.path.expanduser(path)
        return os.path.abspath(path)


    ## Add a connection to the internal FileTracker object
    #  @param bag_path The path of the bag file to use as the key
    #  @param connections_info The information to use as the value
    def add_connection(self, bag_path, connections_info):
        # Use final_bag_path as key in persisitant dictionary
        if not self._ros_connections.get(bag_path):
            self._ros_connections[bag_path] = {}

        self._ros_connections[bag_path].update(connections_info.topics)


    ## Update the ros connections information of a specific capture folder
    #  @param capture_folder The folder to update the connection information in
    def update_capture_ros_connections(self, capture_folder):
        clean_capture_folder = ConnectionsGenerator._clean_path(capture_folder)
        print('clean_capture_folder: {}'.format(clean_capture_folder))
        capture_files = glob.glob('{}/*.bag'.format(clean_capture_folder))
        if len(capture_files) <= 0:
            return

        pattern = '{}[/_]'.format(os.path.basename(clean_capture_folder))
        regex = re.compile(pattern, re.IGNORECASE)
        keys = map(
            lambda path: ConnectionsGenerator._get_file_key(path, regex),
            capture_files
        )

        if os.path.isfile(
                os.path.join(
                    clean_capture_folder,
                    ConnectionsGenerator.CONNECTIONS_FILENAME)
            ):
            with open(
                    os.path.join(
                        clean_capture_folder,
                        ConnectionsGenerator.CONNECTIONS_FILENAME),
                    'r'
            ) as connection_file:
                ros_connections = json.load(connection_file)
        else:
            ros_connections = {}

        for new_key, old_key in keys:
            if old_key in self._ros_connections.keys():
                ros_connections[new_key] = self._ros_connections[old_key]

        with open(
                os.path.join(
                    clean_capture_folder,
                    ConnectionsGenerator.CONNECTIONS_FILENAME),
                'w'
        ) as connection_file:
            json.dump(ros_connections, connection_file)


    ## Remove the unecessary data from persistant storage of the FileTracker
    def clean_up_persistant(self, directory):
        bags = glob.glob('{}/*.bag'.format(directory))
        keys_copy = self._ros_connections.copy().keys()
        for key in keys_copy:
            if key not in bags:
                del self._ros_connections[key]


    @staticmethod
    def _get_file_key(path, regex):
        return (path, regex.sub('', path))
