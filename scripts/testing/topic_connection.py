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

import pprint


## Hold information about topic connections
#  Keep track of the number of connections to a specific topic
#  and the previous message keys for each connection seen as
#  the messages in a bag are iterated
class TopicConnection():

    ## The constructor
    #  @param topic The topic of this topic connection
    #  @param connections The number of connections to this topic
    #  @param special_topic Whether or not this is a special topic
    def __init__(self, topic, connections, special_topic=False):
        self.topic = topic
        self.connections = connections
        if special_topic:
            self.keys = {}
        else:
            self.keys = []


    def __str__(self):
        return 'topic: {}, connections: {}, keys: {}'.format(
            self.topic, self.connections, pprint.pformat(self.keys))


    ## Add a value to self.keys
    #  @param value The value to add to self.keys
    def add_key(self, value):
        if isinstance(self.keys, list):
            self.keys.append(value)


    ## Set the value of self.keys at the index or key provided
    #  @param index The index or key to update
    #  @param value The value to update the index or key with
    def set_key(self, index, value):
        self.keys[index] = value


    ## Get the value from self.keys at the index or key provided
    #  @param index The index or key of the value to get from self.keys
    #  @returns The message key from self.keys
    def get_key(self, index):
        if isinstance(self.keys, list):
            return self.keys[index]
        return self.keys.get(index)


    ## Reset this object
    def reset(self):
        self.keys.clear()
