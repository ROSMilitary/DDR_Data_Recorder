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


import glob
import hashlib
import os
import re
import subprocess


PATTERN = r'^(?P<major_num>\d+)_.*_(?P<minor_num>\d+).bag$'
MAINBAGS_PATTERN = r'^DDR_MAINBAGS_(?P<minor_num>\d+).bag(.active)?$'


def get_sorted_files_list(directory):
    files_list = get_files_list(directory)
    sort_files_list(files_list)
    return files_list


def get_files_list(directory):
    return glob.glob('{}/*.bag'.format(directory), recursive=False)


def get_mainbags_files_list(directory):
    return glob.glob('{}/DDR_MAINBAGS*'.format(directory), recursive=False)


def sort_files_list(files_list):
    regex = re.compile(PATTERN, re.IGNORECASE)
    list.sort(files_list, key=lambda file_path: file_key(file_path, regex) or 0)


def sort_mainbags_files_list(files_list):
    regex = re.compile(MAINBAGS_PATTERN, re.IGNORECASE)
    list.sort(files_list, \
        key=lambda file_path: mainbags_file_key(file_path, regex) or 0)


## Takes a standard bag name and provides a key for it for sorting purposes.
# @param file_path str The path to a specific bag file
# @param regex Compiled regex to use to pull our major and minor num.
# @return tuple with ints of major and minor numbers in first and second index
def file_key(file_path, regex):
    match = re.match(regex, os.path.basename(file_path))
    try:
        major_num = int(match.group('major_num'))
        minor_num = int(match.group('minor_num'))
    except (AttributeError, TypeError, ValueError):
        major_num = None
        minor_num = None
    return (major_num, minor_num)


def mainbags_file_key(file_path, regex):
    match = re.match(regex, os.path.basename(file_path))
    try:
        minor_num = int(match.group('minor_num'))
    except (AttributeError, TypeError, ValueError):
        minor_num = None
    return minor_num

## Ensures a directory exists
#  @param dir The directory to check
#  @returns None
def ensure_directory_exists(directory):
    _directory = os.path.expanduser(directory)
    if not os.path.isdir(_directory):
        os.mkdir(_directory)


## Cleans a directory name/path to achieve a standard format
def clean_directory_name(directory):
    clean_directory = os.path.expandvars(directory)
    clean_directory = os.path.expanduser(clean_directory)
    return os.path.abspath(clean_directory)


## Generates the md5 hash of a file passed
def generate_hash(kml_path):
    with open(kml_path) as kml_file:
        kml_hash = hashlib.md5()
        buffer = kml_file.read()
        kml_hash.update(buffer.encode('utf-8'))
        return kml_hash.hexdigest()


## Calculates the amount of space saved using dynamic mode
# changing over using most expensive mode all the time.
# @param directory str The directory to point to for the bags.
# @return int Size of space saved
def size_saving(directory):
    files = get_files_list(directory)
    sizes = [os.path.getsize(file) for file in files]
    if len(sizes) == 0:
        return -1
    return max(sizes) * len(sizes) - sum(sizes)


## Converts an amount of bytes to human-readable string in appropriate units
# @param amount int bytes
# @param precision int The number of decimal points for rounding
# @return str Representation of amount
def bytes_to_human_str(amount, precision=2):
    if amount >= 1e9:
        return str(round(amount / 1e9, precision)) + ' GB'
    if amount >= 1e6:
        return str(round(amount / 1e6, precision)) + ' MB'
    if amount >= 1e3:
        return str(round(amount / 1e3, precision)) + ' KB'
    return str(amount) + ' B'


## TODO Legacy function
## Kills a process when given a list of string words used to
# identify the process
# and the level to kill it with.
# @param process_name list of string The words used to identify the process.
# Words must be single words, not phrases.
def kill_process(process_name):
    ros_command = ["ps", "ax"]
    processes = subprocess.check_output(ros_command).split("\n")
    for line in processes:
        if not all([word in line for word in process_name]):
            continue
        pid = re.search(r"^ (?P<pid>\d+).*$", line).group("pid")
        if pid:
            try:
                subprocess.Popen(["kill", "-SIGINT", pid])
            except subprocess.CalledProcessError:
                print("There was an error killing process ", process_name)


## Gets a file path with a specified number of parent directories
#  @param file_path The full path to a file
#  @param levels The number of parents to list with the filename
#  @returns A file_path with only the number of parent directories specified
def get_path_with_parents(file_path, levels=1):
    if levels < 1:
        return os.path.basename(file_path)

    len_difference = 2 if file_path.startswith('/') else 1
    if levels >= len(file_path.split('/')) - len_difference:
        return file_path

    common = file_path
    for _ in range(levels + 1):
        common = os.path.dirname(common)

    return os.path.relpath(file_path, common)


if __name__ == '__main__':
    print(size_saving('~/ddr_bags'))
    print(bytes_to_human_str(size_saving('~/ddr_bags')))
