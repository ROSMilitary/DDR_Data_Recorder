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

import xml.etree.ElementTree as ET
import os

import rospkg
import rospy

# Instance of rospkg.RosPack to find ros packages
ROS_PACK = rospkg.RosPack()
PKG_DIR = ROS_PACK.get_path('ddr_data_recorder')

class TopicParser:
    def __init__(self, fileName = None):
        self.fileName = fileName
        self.root = ET.parse(fileName).getroot()
        self.groupListNode = self.root.findall("groupList")[0]
        self.conditionListNode = self.root.findall("conditionList")[0]
        self.conditions = {}
        self.buildTopics()
        self.processConditions()
        self.processGroupsNode()
        self.finalizeTopics()
        self.getnsAdd()
        self.getnsSubtract()

    ## Initializes the topics object used by other functions
    # @return topics dictionary A specialized dictionary that holds string keys tied to sets.
    def buildTopics(self):
        self.topics = {}
        self.topics["required"] = set()
        self.topics["add"] = set()
        self.topics["subtract"] = set()
        self.topics["nsAdd"] = set()
        self.topics["nsSubtract"] = set()
        self.topics["only"] = set()
        self.topics["final"] = set()

    ## Processes the conditionList node of the XML.
    def processConditions(self):
        # Populate the dictionary of conditions.
        for node in self.conditionListNode:
            # Make sure we don't already have this condition.
            if node.tag not in self.conditions:
                # Handle the type of the value
                if node.attrib["value"].lower() == "true":
                    self.conditions[node.tag] = True
                elif node.attrib["value"].lower() == "false":
                    self.conditions[node.tag] = False
                else:
                    try:
                        self.conditions[node.tag] = float(node.attrib["value"])
                    except ValueError:
                        self.conditions[node.tag] = node.attrib["value"]

            else:
                print("Multiple same name conditions found.")
                raise TypeError

    ## Processes the groupList node of the XML.
    # @param topics dictionary Custom object to hold all of the topic groups.
    # @return Modifies the self.topics variable.
    def processGroupsNode(self):
        # TODO Error checking on parameters.
        self.buildTopics()
        for node in self.groupListNode:
            if len(node):
                # We're on a non-leaf node
                self.processNonLeafNode(node)
            else:
                # We're on a leaf node
                self.processLeafNode(node)

    ## Processes a leaf node of the xml
    # @param node XML node Contains the list of topics belonging to this node.
    # @return Modifies the self.topics variable.
    def processLeafNode(self, node):
        # GOOD This function is looking good, not checking conditionals.
        add = False
        if "cond" in node.attrib:
            if "match" not in node.attrib:
                if self.conditions[node.attrib["cond"]]:
                    add = True
            elif (node.attrib["match"].lower() == "tag" and
                        node.tag == self.conditions[node.attrib["cond"]]):
                    add = True
        else:
            add = True

        if add and node.text:
            self.topics[node.attrib["type"]] = self.topics[node.attrib["type"]].union(set(node.text.split()))

    ## Processes a non-leaf node of the xml
    # @param group XML node The parent node to leaf nodes.
    # @param topics dictionary Custom object to hold all of the topic groups.
    # @param conditions dictionary Holds values for the variables stored in the XML file.
    # @return Modifies the topics variable.
    def processNonLeafNode(self, group):
        for node in group:
            if "match" in group.attrib and group.attrib["match"] == "tag":
                if "cond" in group.attrib and node.tag in self.conditions[group.attrib["cond"]]:
                    self.processLeafNode(node)


    ## Performs the final set operations to generate the final topics.
    def finalizeTopics(self):
        if self.topics["only"]:
            self.topics["final"] = self.topics["only"].union(self.topics["required"])

        else:
            self.topics["final"] = (self.topics["add"] - self.topics["subtract"]).union(self.topics["required"])

    def getSuperset(self):
        superSet = self.groupListNode.findall('topicGroup')[0]
        supersetTopics = set()
        for node in superSet:
            if node.text:
                supersetTopics.update(node.text.split())
        return self.topics["add"].union(self.topics["required"]).union(supersetTopics)


    ## Getter for string representation of self.topics["final"]
    # @return string self.topics["final"]
    def getFinalTopics(self):
        return " ".join(self.topics["final"])

    ## Getter for nsAdd
    # @return list of topics that will add much more items
    def getnsAdd(self):
        return self.topics["nsAdd"]

    ## Getter for nsSubtract
    # @return list of topics that will exclude much more items
    def getnsSubtract(self):
        return self.topics["nsSubtract"]


    ## Getter for self.conditions
    # @return dictionary self.conditions
    def getConditions(self):
        return self.conditions

    ## Getter for self.topics
    # @return dictionary self.topics
    def getTopics(self):
        return self.topics

    ## Future function placeholder
    def writeXML(self):
        pass

    ## Creates a dictionary of all nodes and subnodes in the xml file @N
    #  Created to be used with the GUI.
    ## @return dictionary group_list_dictionary
    def get_group_list_dictionary(self):
        group_list_dictionary = {}
        for node in self.groupListNode:
            if node.tag == "whitelist":
                continue
            group_list_dictionary[node.tag] = []
            for n in node:
                if n.tag not in group_list_dictionary[node.tag]:
                    group_list_dictionary[node.tag].append(n.tag)
        return(group_list_dictionary)

if __name__ == "__main__":
    tp = TopicParser(os.path.join(PKG_DIR, 'scripts', 'dynamic_recording', rospy.get_param('/topicXML')))
    # tp = TopicParser("kml.xml")
    tp.conditions["topicGroup"] = "idle" #a nonsense example test
    tp.processGroupsNode()
    tp.finalizeTopics()
    print(tp.getFinalTopics())
