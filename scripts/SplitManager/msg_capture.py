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
#DEALINGS IN THE SOFTWARE.#
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
import shutil
import rospy
import rospkg

ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')
MSG_DIR = os.path.join(PKG_DIR, '../../')

## Save rosmsg files in a folder called ros_msgs in /ddr_bags on startup
def launch_message_capture(directory):

    ## Get /ddr_bags directory
    path = os.path.expanduser(directory)

    ## Create ros_msgs directory
    os.makedirs(str(path) + '/ros_msgs', exist_ok=True)

    ## Walk the path and its subdirectories looking for .msg files. @n
    # then copy to ros_msgs folder
    walk_path = MSG_DIR
    print(MSG_DIR)
    for root, dirs, files in os.walk(walk_path): # pylint: disable=W0612
        for file in files:
            if file.endswith(".msg"):
                if not os.path.isfile(str(path) + '/ros_msgs/' + str(file)):
                    shutil.copy(os.path.join(root, file),
                                str(path) + '/ros_msgs/')
    print('All rosmsg files have been saved in'
          ' %s' % directory + '/ros_msgs')



## Function to save rosmsg files in a folder called ros_msgs in shadow @n
# and manual recorded folders
# @param capture Name of the directory to store ros_msg files
def copy_msg_to_capture_folder(directory, capture):

    ## Set path to manual or shadow folder
    path = os.path.join(os.path.expanduser(directory), capture)

    ## Create ros_msgs directory
    os.makedirs(str(path) + '/ros_msgs', exist_ok=True)

    ## Walk the path and its subdirectories looking for .msg files. @n
    # then copy to ros_msgs folder
    walk_path = os.path.expanduser(os.path.join(directory, 'ros_msgs'))

    ## Copy /ddr_bags/ros_msgs dir to the shadow and manual subdirectories
    for root, dirs, files in os.walk(walk_path): # pylint: disable=W0612
        for file in files:
            if not os.path.isfile(str(path) + '/ros_msgs/' + str(file)):
                shutil.copy(os.path.join(root, file),
                            str(path) + '/ros_msgs/')

def main():

    try:
        rospy.init_node('message_capture', anonymous=False)
        ddr_bag_directory = rospy.get_param("directory")
    except NameError:
        ddr_bag_directory = "~/ddr_bags"

    launch_message_capture(ddr_bag_directory)
    walk_path = os.path.expanduser(ddr_bag_directory)
    for root, dirs, files in os.walk(walk_path): # pylint: disable=W0612
        for sub_dir in dirs:
            if not sub_dir == "ros_msgs":
                copy_msg_to_capture_folder(ddr_bag_directory, sub_dir)

if __name__ == "__main__":
    main()
