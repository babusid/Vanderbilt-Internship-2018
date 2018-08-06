#############
# A secondary interface built on top of the primary interface and other codes
# written by James Zhu and Jacob Gloudemans
# to allow the default state to be one other than the robot
# standing still and to conglomerate as many actions as programmed by James and Jacob
# Created by Sidharth Babu  7/12/2018

from interfaces import *
import math
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

    def defaultmovestate(self):
        self.pint.tail_move(0)
        self.pint.head_move()
        self.pint.update_body_vel(self.default_linear, self.default_angular)

    def sensorinterrupt(self):
        while not rospy.is_shutdown():
            # create your if statement based sensor routine here
            if 1 in self.pint.touch_body:
                self.pint.stop_moving()
                self.pint.tail_move()
                self.pint.head_move(1)
                time.sleep(.5)
                self.pint.head_move()

            elif 1 in self.pint.touch_head:
                self.pint.stop_moving()
                self.pint.tail_move()
                self.pint.head_move(0, .25)
                time.sleep(.25)
                self.pint.head_move(0, -.25)
                time.sleep(.25)
                self.pint.head_move()
            # below returns robot to default state

            elif self.pint.sonar_range <= 0.5 and self.pint.sonar_range != 0:
                print(self.pint.sonar_range)
                self.pint.update_body_vel(.5*self.default_linear, self.default_angular)
                self.pint.head_move(0, .2)
                x = self.pint.sonar_range
                time.sleep(.25)
                self.pint.head_move(0, -.2)
                y = self.pint.sonar_range
                if x > y:
                    self.pint.turn(math.pi)
                elif y > x:
                    self.pint.turn(-math.pi)
                else:
                    self.pint.drive_straight(-.2)

            else:
                self.defaultmovestate()
