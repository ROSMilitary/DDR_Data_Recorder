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


import datetime
import collections
import os
import re
import sys

import rospkg

# Instance of rospkg.RosPack to find ros packages
ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')

SCRIPTS_PATH = os.path.join(PKG_DIR, 'scripts')
sys.path.insert(1, SCRIPTS_PATH)
import util # pylint: disable=wrong-import-position


class Report():
    def __init__(self, directory, filename_suffix='_report.md'):
        self._report = None
        self._opened = False
        self._directory = util.clean_directory_name(directory) # pylint: disable=no-member
        if not os.path.isdir(self.directory):
            raise ValueError('"{}" is not a directory.'.format(self.directory))
        self._filename = self._generate_file_name(filename_suffix)


    def __del__(self):
        if self.opened:
            self.close()


    def __enter__(self):
        if not self.opened:
            self.open()
        return self


    def __exit__(self, exc_type, exc_value, exc_traceback):
        if self.opened:
            self.close()


    @property
    def opened(self):
        return self._opened


    @opened.setter
    def opened(self, _):
        raise TypeError('Cannot set property "opened" of "{}".'.format(
            self.__class__.__name__))


    @property
    def directory(self):
        return self._directory


    @directory.setter
    def directory(self, value):
        raise TypeError('Cannot set property "directory" of "{}".'.format(
            self.__class__.__name__))


    @property
    def filename(self):
        return self._filename


    @filename.setter
    def filename(self, value):
        raise TypeError('Cannot set property "filename" of "{}".'.format(
            self.__class__.__name__))


    def _generate_file_name(self, filename_suffix):
        prefix = os.path.basename(self.directory)
        prefix = re.sub(r'\s+', r'_', prefix)
        return prefix.lower() + filename_suffix


    def open(self):
        if self.opened:
            raise IOError('Cannot open a file that is already open.')

        # Open the file.
        self._report = open(
            os.path.join(
                self.directory,
                self.filename
            ),
            mode='w'
        )
        self._opened = True


    def close(self):
        if not self.opened:
            raise IOError('Cannot close a file that is not open.')

        self._report.close()
        self._opened = False


    def write(self, data):
        if not self.opened:
            raise IOError('Could not write file {}.'.format(os.path.join(
                self.directory,
                self.filename
            )))

        self._report.write(str(data))


    def write_line(self, data=''):
        self.write('{}  \n'.format(data))


    def write_unordered_list(self, unordered_list):
        if len(unordered_list) > 0:
            for element in unordered_list:
                self.write_line('- {}'.format(element))
            self.write_line()
        else:
            self.write_line('None')


    def write_spacer(self):
        self.write_line('\n---\n')


class VerificationReport(Report):
    FILENAME_SUFFIX = '_verification_report.md'
    BadTopic = collections.namedtuple('BadTopic', ['topic', 'difference'])


    def __init__(self, directory):
        super().__init__(directory, VerificationReport.FILENAME_SUFFIX)

        self._bad_topics = []
        self._files_list = []
        self.kml_exists = False
        self.chosen_kml_name = ''
        self.chosen_kml_hash = ''
        self.kml_reason = ''

        self.markdown_hash_exists = False
        self.chosen_markdown_name = ''
        self.markdown_hash = ''

        self.matching_hashes = False


    @property
    def files_list(self):
        return self._files_list


    @files_list.setter
    def files_list(self, value):
        self._files_list = value
        util.sort_files_list(self._files_list) # pylint: disable=no-member


    def write_header(self):
        self.write_line('# Report for ROS bag files in directory "{}"'.format(
            self.directory))
        self.write_line('# {}'.format(
            'Failed' if len(self._bad_topics) > 0 else 'Passed'))
        self.write_line('## Date verified: {}'.format(
            datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
        self.write_spacer()


    def write_settings(self, **flags):
        self.write_line('## verify_data.py Command Line Arguments:')
        for key, value in sorted(flags.items()):
            formatted_key = VerificationReport._format_key(key)

            # If the value is a list, print it as an unordered list
            if isinstance(value, list):
                self.write_line('**{}**: '.format(formatted_key))
                self.write_unordered_list(value)
            # Otherwise, just print the value normally
            else:
                self.write_line('**{}**: {}'.format(formatted_key, value))
        self.write_spacer()


    def write_files_verified(self):
        self.write_line('## Files verified:')
        self.write_unordered_list(self.files_list)
        self.write_spacer()


    def write_unverified_topics(self, topics):
        self.write_line('## Topics that cannot be verified:')
        self.write_unordered_list(sorted(topics))
        self.write_spacer()


    def write_bad_topics(self):
        self.write_line('## Bad topics')
        if len(self._bad_topics) > 0:
            for bad_topic in self._bad_topics:
                self.write_line('Topic {} does not line up by {} messages' \
                    .format(bad_topic.topic, bad_topic.difference))
        else:
            self.write_line('None')
        self.write_spacer()


    def write_kml_section(self):
        self.write_line('## KML information')
        if self.kml_exists:
            if self.chosen_kml_name:
                self.write_line('**KML chosen for analysis:** {}'.format(
                    self.chosen_kml_name))
            if self.chosen_kml_hash:
                self.write_line('**Chosen KML hash:** {}'.format(
                    self.chosen_kml_hash))
            if self.kml_reason:
                self.write_line('**Why was this KML file chosen:** {}'.format(
                    self.kml_reason))
            if self.markdown_hash_exists:
                if self.chosen_markdown_name:
                    self.write_line('**Markdown chosen:** {}'.format(
                        self.chosen_markdown_name))
                if self.markdown_hash:
                    self.write_line('**KML hash from markdown:** {}'.format(
                        self.markdown_hash))
            else:
                self.write_line('No markdown hash could be found.')
        else:
            self.write_line('Could not find a KML, cannot verify this capture.')

        self.write_spacer()


    def add_bad_topic(self, bad_topic, difference):
        self._bad_topics.append(
            VerificationReport.BadTopic(bad_topic, difference))


    ## Formats a dictionary key to be printed properly
    #  Takes a snake case string and returns a space separated
    #  and capitalized string
    #  @param key The string to format
    #  @returns String The formatted string
    @staticmethod
    def _format_key(key):
        formatted_key = key.split('_')
        formatted_key = map(str.capitalize, formatted_key)
        return ' '.join(formatted_key)
