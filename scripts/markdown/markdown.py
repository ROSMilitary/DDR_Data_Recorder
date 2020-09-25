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
import sys
import argparse
sysPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(1, sysPath)
from rosbag_reporter import RosbagReport

## Driver Script Example
# Finds all the directories of Events in the ddr_bags Directory
# Creates a markdown file for each event folder
# Forward Slash Matters
def generate_markdown(bag_dir, name):
    markdown_report = RosbagReport(bag_dir, name)

    with markdown_report:
        markdown_report.read_bags()
        markdown_report.write_report()

def main():
    flags = None
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", metavar="directory", \
                    help="Directory to generate markdown",)
    flags, _ = parser.parse_known_args(sys.argv[1:])

    # modify this line to point to your root ddr_bags directory
    bag_dir = os.path.expanduser(flags.directory)
    if not bag_dir.endswith('/'):
        bag_dir += '/'

    markdown_report = RosbagReport(bag_dir, os.path.basename( \
                            os.path.dirname(bag_dir)))

    with markdown_report:
        markdown_report.read_bags()
        markdown_report.write_base()
    print('done')

if __name__ == "__main__":
    main()
