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



import argparse
import glob
import json
import logging
import os
import pprint
import sys
import threading


## @vars Stores program options
FLAGS = None
# The main logger for the file
_LOGGER = None
# The lock for creating the logger
_LOGGER_LOCK = threading.Lock()


## Add positional arguments to parser.
#  Positional arguments are required and do not use a unique identifier to
#  identify them, instead they are differentiated based on their position
#  within the command.
#  @param parser The ArgumentParser object to add the positional arguments to
#  @returns None
def add_positional_arguments(parser):
    parser.add_argument(
        'directory',
        metavar='DIR',
        help='Directory of captures to check connection information on'
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
        dest="loglevel",
        const=logging.DEBUG
    )
    parser.add_argument(
        '-v', '--verbose',
        help='Enables verbose mode',
        action="store_const",
        dest="loglevel",
        const=logging.INFO,
    )
    parser.set_defaults(loglevel=logging.WARNING)


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

        # Add the output handler.
        _handler = logging.StreamHandler(sys.stdout)
        _handler.setFormatter(logging.Formatter(fmt='%(message)s'))
        _handler.setLevel(FLAGS.loglevel)
        logger.addHandler(_handler)

        # Get a lock on the logger
        with _LOGGER_LOCK:
            # Set the global logger
            _LOGGER = logger

    return _LOGGER


def clean_directory_name(directory):
    clean_directory = os.path.expandvars(directory)
    clean_directory = os.path.expanduser(clean_directory)
    return os.path.abspath(clean_directory)


def main():
    argv = [sys.argv[0]] + parse_args(sys.argv[1:])
    logger = get_logger()
    logger.debug('FLAGS=%s', FLAGS)
    logger.debug('argv=%s', argv)

    clean_directory = clean_directory_name(FLAGS.directory)
    sub_directories = glob.glob('{}/*/'.format(clean_directory))
    clean_sub_directories = map(clean_directory_name, sub_directories)

    for sub_directory in clean_sub_directories:
        if os.path.isdir(sub_directory):
            logger.info('Processing subdirectory "%s"', sub_directory)

            connections_file_path = os.path.join(
                sub_directory, 'connections.json')
            if os.path.isfile(connections_file_path):
                with open(connections_file_path, 'r') \
                    as connection_file:
                    connections = json.load(connection_file)

                files_list = glob.glob('{}/*.bag'.format(sub_directory))
                sub_directory_files = set(files_list)
                connection_files = set(connections.keys())

                error_files = sub_directory_files.symmetric_difference(
                    connection_files)

                if len(error_files) == 0:
                    logger.info('Connections for all files were generated!\n')
                else:
                    print('Connection information for "{}" could be invalid, ' \
                        'either connection information was generated for ' \
                        "files that don't exist anymore, or no connection " \
                        'information was generated for files that exist.' \
                        .format(sub_directory))
                    pprint.pprint(list(error_files))
                    print()
            else:
                print(
                    'No connections.json file in "{}"\n'.format(sub_directory))

        else:
            logger.info('"%s" is not a directory.\n', sub_directory)


if __name__ == '__main__':
    main()
