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


import subprocess
import time
import os
from pathlib import Path
import rospy
import roslaunch
import rospkg
import json
from std_msgs.msg import String



class AutomatedTesting():
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.launch = roslaunch.scriptapi.ROSLaunch()

        rospack = rospkg.RosPack()
        ddr_gui_launch = (rospack.get_path("ddr_data_recorder")
                          + "/launch/ddrGui.launch")
        self.launch.parent = roslaunch.parent.ROSLaunchParent(
            self.uuid, [ddr_gui_launch])
        self.start_ddr()
        self.test_event = rospy.get_param("testEvent")
        self.kml = rospy.get_param("topicXML")
        self.bags_dir = rospy.get_param("directory")
        self.init_dir = self.bags_dir + "/"
        self.active_bag = ""
        self.ddr_dir = Path(self.init_dir).expanduser()
        self.pub = rospy.Publisher('/ddr/api', String, queue_size=10)


    ##  Runs an ls command on a given directory defaults to current directory.
    # @param files list List of strings of file names we're expecting.
    # @param directory str The directory we want to check.
    def check_files(self, files=None, directory=''):
        if files is None:
            files = set()
        output = set(subprocess.check_output('ls ' + directory,
                                             shell=True).split())
        if files == output:
            return 'Directory matches expected output.'
        if files.issuperset(output):
            return 'Program\'s output is smaller than expected.'
        if output.issuperset(files):
            return 'Program\'s output is larger than expected.'
        return 'error'

    ## Publishes a GUI event message
    # @param id_val str
    # @param type_val str
    # @param dynamic_record bool
    # @param shadow_record bool
    # @param sleep_time float The amount of time to sleep after executing the
    # command.
    def publish_gui_event(self, id_val='', type_val='', dynamic_record='false',
                          shadow_record='false', sleep_time=0.1, alert_msg=''):
        subprocess.Popen('rostopic pub -1 /ddr/event ' +
                         'ddr_data_recorder/Event "header:\n' +
                         '  seq: 0\n  stamp: {secs: 0, nsecs: 0}\n  ' +
                         'frame_id: \'\'\n' +
                         'eventID: \'' + id_val + '\'\n' +
                         'eventType: \'' + type_val + '\'\n' +
                         'dynamicRecord: ' + dynamic_record + '\n' +
                         'shadowRecord: ' + shadow_record + '\n' +
                         'alertMsg: ' + alert_msg + '"', shell=True)
        time.sleep(sleep_time)


    ## Publish a message.
    # @param val
    def publish_mode_switch(self, mode=0, sleep_time=0.1):
        # idle - 0
        # teleop_manual - 1
        # waypoint_following - 2
        # ito - 10
        # estop - 11

        translated_mode = ""
        if mode == 1:
            translated_mode = "idle"
        elif mode == 2:
            translated_mode = "teleop"
        elif mode == 3:
            translated_mode = "waypoint_plan_executing"
        elif mode == 11:
            translated_mode = "manual"
        elif mode == 12:
            translated_mode = "estop"
        else:
            translated_mode = "kickstart"


        event = {}
        event['eventID'] = "topicGroup"
        event['dynamicRecord'] = True
        event['shadowRecord'] = False
        event['eventType'] = translated_mode
        event['alertMsg'] = "switching to: " + translated_mode
        self.pub.publish(json.dumps(event))
        time.sleep(sleep_time)

    def trigger_manual_record(self, msg):
        self.publish_gui_event(id_val='button press', type_val='manual record',
                               alert_msg=msg)
        time.sleep(0.2)

    def trigger_shadow_record(self):
        self.publish_gui_event(id_val='button press', type_val='shadow record',
                               alert_msg="Shadow record captured!")
        time.sleep(0.2)

    def mode_switching_test(self, sleep_time=15):
        print("\nSTARTING MODE SWITCHING\n")
        # kick start
        self.publish_mode_switch(11, sleep_time)
        time.sleep(5)
        self.trigger_manual_record("Mode Switching")

        # 15 seconds allows for 3 bags to be recorded before a mode switch
        self.publish_mode_switch(11, sleep_time)
        self.publish_mode_switch(1, sleep_time)
        self.publish_mode_switch(2, sleep_time)
        self.publish_mode_switch(3, sleep_time)
        self.publish_mode_switch(2, sleep_time)
        self.publish_mode_switch(1, sleep_time)
        self.publish_mode_switch(11, sleep_time)
        self.publish_mode_switch(3, sleep_time)
        self.publish_mode_switch(1, sleep_time)
        self.trigger_manual_record("Mode Switching")

    def fast_mode_switching(self, sleep_time=0.2):
        print("\nSTARTING FAST MODE SWITCHING\n")
        self.trigger_manual_record(" Fast Mode Switching")
        time.sleep(1)

        # switch between manual and idle
        self.publish_mode_switch(11, sleep_time)
        self.publish_mode_switch(1, sleep_time)
        self.publish_mode_switch(11, sleep_time)
        self.publish_mode_switch(1, sleep_time)
        # switch between waypoint and teleop
        self.publish_mode_switch(3, sleep_time)
        self.publish_mode_switch(2, sleep_time)
        self.publish_mode_switch(3, sleep_time)
        self.publish_mode_switch(2, sleep_time)
        # switch between idle and teleop
        self.publish_mode_switch(1, sleep_time)
        self.publish_mode_switch(2, sleep_time)
        self.publish_mode_switch(1, sleep_time)
        self.publish_mode_switch(2, sleep_time)
        #time.sleep(1)

        self.trigger_manual_record(" Fast Mode Switching")
        time.sleep(2)

    def ddr_trigger_fast_mode_switching(self):
        print("\nSTARTING DDR TRIGGER FAST MODE SWITCHING\n")
        self.trigger_manual_record("DDR Fast Mode Switching")
        time.sleep(2)
        # switch between manual and idle
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='idle', alert_msg="idle")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='manual', alert_msg="manual")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='idle', alert_msg="idle")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='manual', alert_msg="manual")
        # switch between waypoint and teleop
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='waypoint_plan_executing',
                               alert_msg="waypoint")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='teleop', alert_msg="teleop")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='waypoint_plan_executing',
                               alert_msg="waypoint")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='teleop', alert_msg="teleop")
        # switch between idle and teleop
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='idle', alert_msg="idle")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='teleop', alert_msg="teleop")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='idle', alert_msg="idle")
        self.publish_gui_event(dynamic_record='true', id_val='topicGroup',
                               type_val='teleop', alert_msg="teleop")
        self.trigger_manual_record("DDR Fast Mode Switching")
        time.sleep(1)

    def wait_for_next_active_bag(self):
        for bag in os.listdir(str(self.ddr_dir)):
            if bag.endswith(".bag.active"):
                active_bag = bag
                next_bag = bag
                # wait until the next active bag appears in the folder
                while active_bag == next_bag:
                    for bag1 in os.listdir(str(self.ddr_dir)):
                        if bag1.endswith(".bag.active"):
                            next_bag = bag1

    def operational_capture_testing(self):
        print("\nSTARTING OPERATIONAL CAPTURE TESTING\n")
        # switch to idle and let 2 bags accumulate then switch to manual
        # and let two more bags accumulate
        self.publish_mode_switch(1, 10) # idle mode
        self.publish_mode_switch(11, 10) # manual mode
        self.trigger_manual_record("Operational Capture Testing")
        time.sleep(15) # allow 3 manual bags to accumulate
        self.publish_mode_switch(1, 15)
        self.publish_mode_switch(11, 15) # manual mode

        self.wait_for_next_active_bag()

        self.trigger_shadow_record()
        self.trigger_manual_record("Operational Capture Testing")

    def single_bag_capture(self):
        print("\nSTARTING SINGLE BAG CAPTURE TESTING\n")
        self.publish_mode_switch(11, 10) # manual mode
        self.wait_for_next_active_bag()
        self.trigger_manual_record("Single Bag Capture")
        self.trigger_manual_record("Single Bag Capture")

    def multi_bag_capture(self):
        print("\nSTARTING MULTI BAG CAPTURE TESTING\n")
        self.publish_mode_switch(11, 10) # manual mode
        self.wait_for_next_active_bag()
        self.trigger_manual_record("Multi Bag Capture_1")
        self.trigger_manual_record("Multi Bag Capture_1")
        self.trigger_manual_record("Multi Bag Capture_2")
        self.trigger_manual_record("Multi Bag Capture_2")
        self.trigger_manual_record("Multi Bag Capture_3")
        self.trigger_manual_record("Multi Bag Capture_3")
        self.wait_for_next_active_bag()
        self.trigger_manual_record("Multi Bag Capture_4")
        self.trigger_manual_record("Multi Bag Capture_4")
        self.trigger_manual_record("Multi Bag Capture_5")
        self.trigger_manual_record("Multi Bag Capture_5")
        self.trigger_manual_record("Multi Bag Capture_6")
        self.trigger_manual_record("Multi Bag Capture_6")

    def shadow_stress_testing(self):
        print("\nSTARTING SHADOW STRESS TESTING\n")
        self.trigger_manual_record("Shadow Stress Test")
        self.wait_for_next_active_bag()
        self.trigger_shadow_record()
        self.trigger_shadow_record()
        self.trigger_shadow_record()
        self.trigger_shadow_record()
        self.trigger_manual_record("Shadow Stress Test")

    def dynamic_record_testing(self):

        #configuration check
        if self.test_event is not False or self.kml != "/kml.xml":
            print("WARNING - rosparams not configured correctly. Did you:")
            print("1. set test event mode to 'false'?")
            print("2. Use the kml.xml?")
            return

        #warning statements
        print("\nYOU ARE RUNNING PART 1 OF THE AUTOMATED TEST SCRIPT\n")
        print("Remember - DDR must NOT be running before beginning part 1 \
AND the ddr_bags directory must be empty!\n\n\n")

        #run tests for part 1
        self.mode_switching_test()
        self.fast_mode_switching(0.2)
        self.ddr_trigger_fast_mode_switching()

        #completion statements
        print("\n\nAutomated Testing Part 1 Complete!")
        print("Part 2 of DDR testing is about to begin - 15 seconds!.\n\n")
        time.sleep(15)

    def capture_verification_testing(self):
        #configuration check
        if self.test_event is not True or self.kml != "/testkml.xml":
            print("WARNING - rosparams not configured correctly. Did you:")
            print("1. change test event mode to 'false'?")
            print("2. delete the testkml.xml?")
            return

        #warning statements
        print("\n\nYOU ARE RUNNING PART 2 OF THE AUTOMATED TEST SCRIPT\n")
        print("\nRemember - DDR must NOT be running before part 2!\n\n")

        #run tests for part 2
        self.operational_capture_testing()
        self.single_bag_capture()
        self.multi_bag_capture()
        self.shadow_stress_testing()

        #completion statements
        print("\nAutomated Testing Part 2 Complete!")
        time.sleep(10)

    def start_ddr(self):
        rospy.init_node('ddr_verification_scripts', anonymous=True)
        roslaunch.configure_logging(self.uuid)
        self.launch.start()
        rospy.loginfo("ddr started")
        time.sleep(5)

    def kill_dynamic_recorder(self):
        node = "/dynamic_recorder"
        os.system("rosnode kill " + node)
        time.sleep(1)

    def kill_record_process(self):
        os.system("killall record")

    def kill_split_manager(self):
        node = "/split_manager"
        os.system("rosnode kill " + node)
        time.sleep(1)

    def start_dynamic_recorder(self):
        rospy.set_param("topicXML", "/testkml.xml")
        rospy.set_param("testEvent", True)
        time.sleep(1)
        self.kml = rospy.get_param("topicXML")
        self.test_event = rospy.get_param("testEvent")
        time.sleep(1)

        package = "ddr_data_recorder"
        executable = "dynamic_recorder.py"
        node = roslaunch.core.Node(package, executable)
        self.launch.launch(node)
        time.sleep(2)

    def start_split_manager(self):
        package = "ddr_data_recorder"
        executable = "manager.py"
        node = roslaunch.core.Node(package, executable)
        self.launch.launch(node)
        time.sleep(2)

    def delete_loose_bags(self):
        for bag in os.listdir(str(self.ddr_dir)):
            if bag.endswith(".bag"):
                os.remove(os.path.join(str(self.ddr_dir), bag))

    def main(self):
        time.sleep(5)

        welcome_message = """\n\n\nWelcome to the DDR Automated Testing Tool - Beta Version!
Remember to stop DDR before clearing the DDR bags folder.
Before beginning either phase of the automated testing:
1) Remember DDR should NOT be running
2) An EMPTY ddr_bags directory.
Good Luck!\n\n"""

        print(welcome_message)

        # Dynamic Recorder Testing
        """
        CONFIGURATION

        KML:        kml.xml
        testEvent:  FALSE
        """
        self.dynamic_record_testing()


        # Capture Verification Testing
        """
        CONFIGURATION

        KML:        testkml.xml
        testEvent:  TRUE
        """
        self.kill_dynamic_recorder()
        self.kill_record_process()
        self.kill_split_manager()
        self.delete_loose_bags()
        self.start_dynamic_recorder()
        self.start_split_manager()
        self.wait_for_next_active_bag()
        self.capture_verification_testing()

        exit()

if __name__ == '__main__':
    DDR_AUTO_TEST_SCRIPTS = AutomatedTesting()
    DDR_AUTO_TEST_SCRIPTS.main()
