#############
# A secondary interface built on top of the primary interface
# written by James Zhu and Jacob Gloudemans
# to allow the default state to be one other than the robot standing still
# and to have a basic sensor driven loop
# Created by Sidharth Babu  7/12/2018

from interfaces import*


class SecondaryInterface:

    def __init__(self, robotname, linear, angular):
        self.default_linear = linear
        self.default_angular = angular
        self.pint = primary_interface(robotname)
        self.pint.update_body_vel(linear,angular)

    def defaultstate(self):
        self.pint.update_body_vel(self.default_linear, self.default_angular)




    def sensorinterrupt(self):
        if 1 in self.pint.touch_body:
            # What to do if someone touches the body
            pass
        elif 1 in self.pint.touch_head:
            # What to do if someone touches the head
            pass
        else:
            self.defaultstate()
