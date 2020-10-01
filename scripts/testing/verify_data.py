#!/usr/bin/env python3

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


import argparse
import collections
import glob
import logging
import logging.handlers
import mmap
import os
import pprint
import re
import sys
import threading

import rosbag
import rospkg
from topic_bag_checker import TopicBagChecker
from topic_connection import TopicConnection
from verification_report import VerificationReport

# Instance of rospkg.RosPack to find ros packages
ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')

SCRIPTS_PATH = os.path.join(PKG_DIR, 'scripts')
sys.path.insert(1, SCRIPTS_PATH)

import util # pylint: disable=wrong-import-position


## @vars Stores program options
FLAGS = None
## @vars The list of topics not to verify
ILLEGAL_TOPICS = [
    '/your/topic'
]
## @vars The list of topics which require a
#  special message key comparison function
SPECIAL_TOPICS = [
    '/your/topic'
]
# The main logger for the file
_LOGGER = None
# The lock for creating the logger
_LOGGER_LOCK = threading.Lock()
# Named tuple representing a key of a particular message
MessageKey = collections.namedtuple(
    'MessageKey', ['seq', 'secs', 'nsecs'])
# Named tuple representing a key of /tf messages
TransformKey = collections.namedtuple(
    'TransformKey', ['seq', 'secs', 'nsecs', 'child_frame_id'])

CURRENT_REPORT = None


## Add positional arguments to parser.
#  Positional arguments are required and do not use a unique identifier to
#  identify them, instead they are differentiated based on their position
#  within the command.
#  @param parser The ArgumentParser object to add the positional arguments to
#  @returns None
def add_positional_arguments(parser):
    parser.add_argument(
        'directories',
        metavar='directory',
        nargs='+',
        help='Directories of bag files to postprocess'
    )


## Add optional arguments to parser.
#  Optional arguments are not required and use a unique identifier to
#  identify them.
#  @param parser The ArgumentParser object to add the optional arguments to
#  @returns None
def add_optional_arguments(parser):
    parser.add_argument(
        '-d', '--debug',
        help=argparse.SUPPRESS,
        action="store_const",
        dest="log_level",
        const=logging.DEBUG
    )
    parser.add_argument(
        '-v', '--verbose',
        help='Enables verbose mode',
        action="store_const",
        dest="log_level",
        const=logging.INFO,
    )
    parser.add_argument(
        '-t', '--tolerance',
        type=int,
        default=10,
        metavar='T',
        help='The tolerance to decide if a gap is missing data, ' \
            'or a different publisher'
    )
    parser.add_argument(
        '--log-name',
        default='verify_data.log',
        metavar='NAME',
        help='The name of the log files'
    )
    parser.add_argument(
        '--log-dir',
        default=util.clean_directory_name('~/ddr_bags/verify_data_logs/'), # pylint: disable=no-member
        metavar='DIR',
        help='The directory to store the verify data log files in'
    )
    parser.add_argument(
        '--num-logs',
        type=int,
        default=10,
        metavar='NUM',
        help='The number of log files to keep'
    )
    parser.add_argument(
        '--log-size',
        type=int,
        default=1000000,
        metavar='B',
        help='The size of each log file in bytes ' \
             'before generating a new log file'
    )
    parser.add_argument(
        '--kml-path',
        metavar='PATH',
        help='The path to the kml file to use for verification'
    )
    parser.set_defaults(log_level=logging.WARNING)


## Parse all positional arguments and options.
#  Parse the positional arguments and options specified into the global FLAGS
#  variable and store unknown arguments into a new argv to pass to main.
#  @param argv An argument vector to parse
#  @returns A new argv for use in main
def parse_args(argv):
    global FLAGS #pylint: disable=global-statement

    # Argument parser from argparse
    parser = argparse.ArgumentParser()

    # Add all of the positional arguments
    add_positional_arguments(parser)
    # Add all of the optional arguments
    add_optional_arguments(parser)

    # Parse the arguments into FLAGS and argv
    FLAGS, _argv = parser.parse_known_args(argv)

    return _argv


## Get the main logger.
#  Get the main logger or create a main logger if one does not exist
#  @returns The main logger
def get_logger():
    global _LOGGER #pylint: disable=global-statement

    # Check if the logger has already been created
    if not _LOGGER:
        # Get a logger and name it
        logger = logging.getLogger('main')

        # Set the logging level for the current program
        logger.setLevel(logging.DEBUG)

        # Clean user input and make sure the directory exists
        log_dir = util.clean_directory_name(FLAGS.log_dir) # pylint: disable=no-member
        util.ensure_directory_exists(log_dir) # pylint: disable=no-member

        # Clean user input and get just the name
        if FLAGS.log_name.endswith('/'):
            log_name = os.path.dirname(FLAGS.log_name)
        else:
            log_name = os.path.basename(FLAGS.log_name)

        # Add the output handler.
        _file_handler = logging.handlers.RotatingFileHandler(
            os.path.join(log_dir, log_name),
            maxBytes=FLAGS.log_size,
            backupCount=FLAGS.num_logs
        )
        _file_handler.setFormatter(logging.Formatter(
            fmt='%(asctime)s.%(msecs)03d %(message)s',
            datefmt='%Y_%m_%d %H:%M:%S'
        ))
        _file_handler.setLevel(logging.DEBUG)
        logger.addHandler(_file_handler)

        # Add the output handler.
        _handler = logging.StreamHandler(sys.stdout)
        _handler.setFormatter(logging.Formatter(fmt='%(message)s'))
        _handler.setLevel(FLAGS.log_level)
        logger.addHandler(_handler)

        # Get a lock on the logger
        with _LOGGER_LOCK:
            # Set the global logger
            _LOGGER = logger

    return _LOGGER


## Determine if a message is valid
#  Determine if a message is valid to be used for indexing
#  in an old bag
#  @param message The message to verify
#  @returns Bool if message is valid for use in indexing in an old bag
def is_valid(value):
    if value.topic not in ILLEGAL_TOPICS:
        # Message must have a header field
        # This is how to check for a header according
        # to the ROS Cookbook and it's twice as fast
        # as hasattr(value.message, 'header') so ¯\_(ツ)_/¯
        if value.message._has_header: # pylint: disable=protected-access
            return True

    return False


## Get the key from a bag message
#  Gets the unique key for a message from a bag
#  @param bag_message The bag message to get the unique key for
#  @returns collections.namedtuple MessageKey or TransformKey that represents
#  the unique key value of the bag message
def get_message_key(bag_message):
    logger = get_logger()
    key = None

    # Skip illegal topics
    if bag_message.topic not in ILLEGAL_TOPICS:
        # Handle /ddr/andBeyond topic
        if bag_message.topic == '/ddr/andBeyond':
            key = bag_message.message.data

        # Handle /special topic
        # use this template to create a handler for topics that have weird data vars
        elif bag_message.topic == '/special':
            transform = bag_message.message.transforms[0]
            key = TransformKey(transform.header.seq,
                               transform.header.stamp.secs,
                               transform.header.stamp.nsecs,
                               transform.child_frame_id)



        # ***********************************

        elif is_valid(bag_message):
            # Message key is a tuple of the sequence number and timestamp
            # from the header of the message
            key = MessageKey(bag_message.message.header.seq,
                             bag_message.message.header.stamp.secs,
                             bag_message.message.header.stamp.nsecs)

        else:
            logger.info('No handler for topic %s', bag_message.topic)
            logger.debug('message: %s\n', bag_message.message)

    return key


## Handle comparison of the special case /andBeyond topic
#  @param current_key The key of the current message to compare
#  @param key The key stored in the list of previous keys to compare against
#  @returns int The value of the comparison between current_key and key
def and_beyond_handler(current_key, key):
    diff = current_key - key
    if diff > 0:
        return diff
    return None


## Handle comparison of the special case /special topic
#  @param current_key The key of the current message to compare
#  @param key The key stored in the list of previous keys to compare against
#  @returns Bool The value of the comparison between current_key and key
def special_handler(current_key, key):
    key_is_good = False

    if current_key.secs - key.secs > 0:
        key_is_good = True
    elif current_key.secs - key.secs == 0:
        if current_key.nsecs - key.nsecs > 0:
            key_is_good = True

    return key_is_good




## Compares keys of messages, calling the special case handlers if needed
#  @param bag_message The message from the bag
#  @param current_key The key of the current message to compare
#  @param key The key stored in the list of previous keys to compare against
#  @returns int, Bool, or None The value of the comparison between
#  current_key and key
def compare_keys(bag_message, current_key, key):
    logger = get_logger()

    # Skip illegal topics
    if bag_message.topic in ILLEGAL_TOPICS:
        logger.debug('Illegal topic: %s', bag_message.topic)
        return None

    ret_val = None
    # Handle /ddr/andBeyond topic
    if bag_message.topic == '/ddr/andBeyond':
        ret_val = and_beyond_handler(current_key, key)

    # Handle /special topic
    elif bag_message.topic == '/special':
        ret_val = special_handler(current_key, key)


    elif is_valid(bag_message):
        ret_val = current_key.seq - key.seq

    return ret_val


## Handle the verification of any special case topics that require
#  different key verification logic
#  @param bag_message The message from the bag
#  @param topic_connection The TopicConnection object containing topic keys
#  and number of connections
#  @param current_key The key of the current message to compare
def verify_special(bag_message, topic_connection, current_key):
    logger = get_logger()

    if bag_message.topic == '/special':
        key = topic_connection.get_key(current_key.child_frame_id)
        if key is not None:
            if not tf_handler(current_key, key):
                print('Topic {} may not line up.'.format(bag_message.topic))
                logger.debug('current_key: %s\n' \
                             'key: %s',
                             current_key,
                             key)
        logger.debug('verify_special topic_connection: %s, current_key: %s',
                     topic_connection, current_key)
        topic_connection.set_key(current_key.child_frame_id, current_key)




## Handle the verification of all topics not requiring special case logic
#  @param bag_message The message from the bag
#  @param topic_connection The TopicConnection object containing topic keys
#  and number of connections
#  @param current_key The key of the current message to compare
def verify(bag_message, topic_connection, current_key):
    logger = get_logger()

    if bag_message.topic in SPECIAL_TOPICS:
        verify_special(bag_message, topic_connection, current_key)
        return

    if len(topic_connection.keys) == 0:
        topic_connection.add_key(current_key)
        return

    lowest_diff = None
    index = -1
    for i, key in enumerate(topic_connection.keys):
        diff = compare_keys(bag_message, current_key, key)
        if diff is not None:
            if lowest_diff is None or diff < lowest_diff:
                lowest_diff = diff
                index = i

    if lowest_diff is None:
        logger.debug('lowest_diff is None.\n' \
                     'current_key: %s;\n' \
                     'topic_connection: %s',
                     current_key,
                     topic_connection)
        return

    if lowest_diff == 1:
        topic_connection.set_key(index, current_key)
        logger.debug('lowest_diff == 1.\ncurrent_key: %s\nkey: %s',
                     current_key,
                     topic_connection)
    elif lowest_diff <= 0 and len(topic_connection.keys) < \
        topic_connection.connections:
        topic_connection.add_key(current_key)
        logger.debug('lowest_diff: %s index: %s topic: %s',
                     lowest_diff, index, bag_message.topic)
        logger.debug('lowest_diff <= 0 and len(topic_connection.keys) < ' \
                     'topic_connection.connections.\ncurrent_key: %s\nkey: %s',
                     current_key,
                     topic_connection)
    elif lowest_diff > FLAGS.tolerance and len(topic_connection.keys) < \
        topic_connection.connections:
        topic_connection.add_key(current_key)
        logger.debug('lowest_diff: %s index: %s topic: %s',
                     lowest_diff, index, bag_message.topic)
        logger.debug('lowest_diff > FLAGS.tolerance and ' \
                     'len(topic_connection.keys) < ' \
                     'topic_connection.connections.\ncurrent_key: %s\nkey: %s',
                     current_key,
                     topic_connection)
    elif lowest_diff <= FLAGS.tolerance:
        logger.debug('lowest_diff: %s index: %s topic: %s',
                     lowest_diff, index, bag_message.topic)
        print('Topic {} does not line up by {} messages'.format(
            bag_message.topic, lowest_diff))
        CURRENT_REPORT.add_bad_topic(bag_message.topic, lowest_diff)
        logger.debug('current_key: %s\n' \
                     'key: %s',
                     current_key,
                     topic_connection.get_key(index))
        topic_connection.set_key(index, current_key)


## Verify the contents of a single file
#  @param file The path to the file to verify
#  @param topic_connections All topic connections
def verify_file(file, topic_connections):
    with rosbag.Bag(file, 'r') as bag:
        bag_info = bag.get_type_and_topic_info()
        topic_connections[file] = set(bag_info.topics.keys())
        for topic, value in bag_info.topics.items():
            if topic in ILLEGAL_TOPICS:
                continue
            topic_connection = topic_connections.get(topic)
            if topic_connection:
                topic_connection.connections = value.connections
            else:
                if topic in SPECIAL_TOPICS:
                    topic_connections[topic] = TopicConnection(
                        topic, value.connections, special_topic=True)
                else:
                    topic_connections[topic] = TopicConnection(
                        topic, value.connections)

        messages = bag.read_messages()
        for bag_message in messages:
            if bag_message.topic in ILLEGAL_TOPICS:
                continue
            current_key = get_message_key(bag_message)
            verify(
                bag_message,
                topic_connections[bag_message.topic],
                current_key
            )


## Reset the keys for specific topics
#  @param topic_connections All topic connections
#  @param topics_to_clear The list of topics to clear the keys of
def reset_topics(topic_connections, topics_to_clear):
    for topic in topics_to_clear:
        topic_connection = topic_connections.get(topic)
        if topic_connection is not None:
            topic_connection.reset()


## Verify all the files in a directory
#  @param files_list The list of file paths to verify
#  @param tbc The TopicBagChecker to use for verification of bags
#  and clearing of keys
def verify_files_in_directory(files_list, tbc):
    logger = get_logger()
    mode_pattern = r'^[a-zA-Z0-9_ -/()]*\d+_(?P<mode>.*)_\d+\.bag$'

    topic_connections = {}
    previous_mode = None
    for file in files_list:
        logger.info('Processing file %s', file)
        if previous_mode is not None:
            match = re.match(mode_pattern, file)
            if match is not None:
                current_mode = match.group('mode')
                if current_mode != previous_mode:
                    topics_to_clear = tbc.process_bag(previous_mode,
                                                      current_mode)
                    reset_topics(topic_connections, topics_to_clear)
                previous_mode = current_mode
        else:
            match = re.match(mode_pattern, file)
            if match:
                previous_mode = match.group('mode')

        logger.debug('previous_mode: %s', previous_mode)
        verify_file(file, topic_connections)


## Gets the path to the kml file
#  Gets the path to the kml file using the kml-file command line argument first.
#  If no command line parameter is found or it is invalid, it looks for a kml
#  file in the directory that is currently being processed. If there is still no
#  kml file found, the final place checked is in the ddr_data_director package
#  @param directory The current directory being verified
#  @returns String The path to the kml file
def get_kml_paths(directory):
    logger = get_logger()
    kml_glob_pattern = '{}/*.xml'

    # Check if the user specified a kml file
    if FLAGS.kml_path:
        if not FLAGS.kml_path.endswith('/'):
            kml_path = os.path.expanduser(FLAGS.kml_path)
            if os.path.isfile(kml_path):
                if CURRENT_REPORT is not None:
                    CURRENT_REPORT.kml_exists = True
                    CURRENT_REPORT.chosen_kml_name = os.path.basename(kml_path)
                    CURRENT_REPORT.kml_reason = \
                        'User specified a KML file for verification'
                return [kml_path]
        logger.info(
            '"%s" is not a valid kml path. Attempting to find a kml in "%s"',
            kml_path,
            directory
        )

    # User did not specify a kml, attempt to find one in the current directory
    kml_paths = glob.glob(kml_glob_pattern.format(directory))
    if kml_paths:
        CURRENT_REPORT.kml_exists = True
        CURRENT_REPORT.kml_reason = 'KML file was found in capture folder'
        return kml_paths

    # No kml in current directory, attempt to find one
    # in the ddr_data_recorder ros package
    ddr_dir = os.path.join(
        SCRIPTS_PATH,
        'dynamic_recording'
    )
    logger.info(
        'No kml file in "%s". Attempting to find a kml in "%s"',
        directory,
        ddr_dir
    )
    kml_paths = glob.glob(kml_glob_pattern.format(ddr_dir))
    if kml_paths:
        CURRENT_REPORT.kml_exists = True
        CURRENT_REPORT.kml_reason = 'KML file was not found in capture ' \
            'folder, but was found in the installation directory'
        return kml_paths

    logger.info('No kml files found.')
    return []


## Gets the path to the markdown file
#  Gets the path to the markdown file in the curent directory. If multiple files
#  exist, one is picked at random (due to how glob works).
#  @param directory The current directory being verified
#  @returns String The path to the markdown file
def get_markdown_paths(directory):
    markdown_paths = glob.glob(os.path.join(directory, '*.md'))
    if len(markdown_paths) == 1:
        return markdown_paths[0]

    if markdown_paths:
        return markdown_paths

    logger = get_logger()
    logger.info('No markdown files found in "%s"', directory)
    return []


## Gets the kml hash from the markdown file
#  Looks through the markdown file and finds the line that contains the hash of
#  the kml file used when it was created
#  @param markdown_path The path to the markdown file to get the hash from
#  @returns String The hash of the kml from the markdown
def get_markdown_hash(markdown_path):
    logger = get_logger()
    try:
        with open(markdown_path, 'rb', 0) as _file, \
            mmap.mmap(_file.fileno(), 0, access=mmap.ACCESS_READ) \
            as markdown_file:
            search_result = re.search(
                br'## Hash of Used XML file \(md5\): (?P<hash>[0-9a-f]{32})',
                markdown_file
            )
            if search_result:
                markdown_hash = search_result.group('hash').decode('utf-8')
                if CURRENT_REPORT is not None:
                    CURRENT_REPORT.markdown_hash_exists = True
                    CURRENT_REPORT.markdown_hash = markdown_hash
                logger.info('Hash for kml found in "%s"', markdown_path)
                return markdown_hash
    except ValueError:
        pass

    logger.info('No hash for kml found in "%s"', markdown_path)
    return ''


## Verifies that the hash of the kml file and the hash from the markdown file
#  match
#  @param kml_path The path to the kml file to verify
#  @param markdown_path The path to the markdown file to verify the kml against
#  @returns Bool True if the hashes match, False if they do not
def verify_kml_hash(kml_paths, markdown_paths):
    logger = get_logger()

    for markdown_path in markdown_paths:
        markdown_kml_hash = get_markdown_hash(markdown_path)
        if markdown_kml_hash:
            logger.debug('markdown_kml_hash = %s', markdown_kml_hash)
            if CURRENT_REPORT is not None:
                CURRENT_REPORT.chosen_markdown_name = \
                    os.path.basename(markdown_path)
            break

    for kml_path in kml_paths:
        current_kml_hash = util.generate_hash(kml_path) # pylint: disable=no-member

        if not markdown_kml_hash:
            CURRENT_REPORT.chosen_kml_name = util.get_path_with_parents( # pylint: disable=no-member
                kml_path, 1)
            CURRENT_REPORT.chosen_kml_hash = current_kml_hash
            return False, kml_path

        if current_kml_hash == markdown_kml_hash:
            logger.debug('kml_hash = %s', current_kml_hash)
            if CURRENT_REPORT is not None:
                CURRENT_REPORT.chosen_kml_name = util.get_path_with_parents( # pylint: disable=no-member
                    kml_path, 1)
                CURRENT_REPORT.chosen_kml_hash = current_kml_hash
            return True, kml_path

    return False, ''


def write_report():
    if CURRENT_REPORT is not None:
        with CURRENT_REPORT:
            CURRENT_REPORT.write_header()
            CURRENT_REPORT.write_settings(**vars(FLAGS))
            CURRENT_REPORT.write_kml_section()
            CURRENT_REPORT.write_files_verified()
            CURRENT_REPORT.write_unverified_topics(ILLEGAL_TOPICS)
            CURRENT_REPORT.write_bad_topics()


def main():
    global CURRENT_REPORT # pylint: disable=global-statement
    argv = [sys.argv[0]] + parse_args(sys.argv[1:])
    logger = get_logger()
    logger.debug('FLAGS=%s', FLAGS)
    logger.debug('argv=%s', argv)
    logger.info('Unable to verify the following topics:\n%s', pprint.pformat(
        ILLEGAL_TOPICS))
    logger.info('Topics requiring special handling:\n%s', pprint.pformat(
        SPECIAL_TOPICS))


    directories = map(os.path.expanduser, FLAGS.directories)

    for directory in directories:
        if os.path.isdir(directory):
            logger.info('Processing directory "%s"', directory)
            CURRENT_REPORT = VerificationReport(directory)

            kml_paths = get_kml_paths(directory)
            if not kml_paths:
                print('Unable to verify "{}". No kml found.'.format(directory))
                if CURRENT_REPORT is not None:
                    CURRENT_REPORT.kml_exists = False
                    write_report()
                continue

            markdown_paths = get_markdown_paths(directory)
            if not markdown_paths:
                print('Unable to verify "{}". No markdown ' \
                      'to compare kml hash against.'.format(directory))
            else:
                hash_verified, kml_path = verify_kml_hash(
                    kml_paths, markdown_paths)
                if CURRENT_REPORT is not None:
                    CURRENT_REPORT.matching_hashes = hash_verified

                if not hash_verified:
                    print('Unable to verify {}. KML and markdown ' \
                        'hashes do not match.'.format(directory))

            tbc = TopicBagChecker(kml_path)
            files_list = util.get_sorted_files_list(directory) # pylint: disable=no-member
            if CURRENT_REPORT is not None:
                CURRENT_REPORT.files_list = files_list
            verify_files_in_directory(files_list, tbc)

        else:
            logger.info('"%s" is not a directory.', directory)

        write_report()


if __name__ == '__main__':
    main()
