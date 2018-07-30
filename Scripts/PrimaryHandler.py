#############
# A secondary interface built on top of the primary interface and other codes
# written by James Zhu and Jacob Gloudemans
# to allow the default state to be one other than the robot 
# standing still and to conglomerate as many actions as programmed by James and Jacob
# Created by Sidharth Babu  7/12/2018

from interfaces import *

import rospy
from std_msgs.msg import String
from miro_msgs.msg import platform_mics, platform_sensors
from array import array
import time
import numpy
from sound_test import sound_test


class SecondaryInterface:

    def __init__(self, robotname, linear, angular):
        self.default_linear = linear
        self.default_angular = angular
        self.pint = primary_interface(robotname)
        self.pint.update_body_vel(linear, angular)

    def defaultmovestate(self):
        self.pint.update_body_vel(self.default_linear, self.default_angular)

    """def sensorinterrupt(self):
        if 1 in self.pint.touch_body:
            self.pint.stop_moving()
            self.pint.tail_move()
        elif 1 in self.pint.touch_head:
            self.pint.stop_moving()
            self.pint.tail_move()
        else:

            self.defaultmovestate()
"""
    def data_in(self):

        start_time = rospy.get_rostime()
        self.old_value = 0.0
        flag_pet = False
        flag_pat = False
        self.pint.body_config_speed = [5, 5, 5, 5]

        while not rospy.is_shutdown():

            if self.pint.touch_body:

                # find average value of body touch
                try:
                    self.value = (self.pint.touch_body[0] + 2.0 * self.pint.touch_body[1] + 3.0 *
                                  self.pint.touch_body[2] + 4.0 * self.pint.touch_body[3]) / (
                                     numpy.sum(self.pint.touch_body))
                except ZeroDivisionError:
                    self.value = 0

                # test for 2 different types of touching (petting and patting)
                if abs(self.value - self.old_value) > .5:

                    # No flags activated
                    if flag_pet == False and flag_pat == False:

                        if self.value != 0:
                            flag_pet = True
                            flag_pat = False
                        else:
                            flag_pet = False
                            flag_pat = True
                        print flag_pet, flag_pat
                        start_time = rospy.get_rostime()
                    # one flag activated
                    elif flag_pet == True:

                        # if next event occurs within 1 sec, initiates response
                        if self.value != 0 and (rospy.get_rostime() - start_time).to_sec() < 1.0:
                            behavior.pet_detect()
                            flag_pet = False
                            time.sleep(.5)
                        else:
                            flag_pet = False
                    elif flag_pat == True:

                        if (rospy.get_rostime() - start_time).to_sec() < 1.0:
                            behavior.pat_detect()
                            flag_pat = False
                            time.sleep(.5)
                        else:
                            flag_pat = False
                self.old_value = self.value
            else:
                self.defaultmovestate()

    def pet_detect(self):
        # nod head in approval and wag tail
        self.pint.stop_moving()
        self.pint.tail_move()
        self.pint.body_config = [0, 0, 0, 1]
        time.sleep(.5)
        self.pint.body_config = [0, 0, 0, 0]

    def pat_detect(self):
        # shake head in disapproval and droop tail
        self.pint.stop_moving()
        self.pint.tail_move(-1)
        self.pint.body_config = [0, 0, .2, 0]
        time.sleep(.25)
        self.pint.body_config = [0, 0, -2, 0]
        time.sleep(.25)
        self.pint.body_config = [0, 0, 0, 0]
