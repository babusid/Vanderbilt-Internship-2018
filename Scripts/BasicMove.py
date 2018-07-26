from interfaces import *
import rospy
import math


class SecondaryInterface():

    def __init__(self, robotname, linear, angular):
        # default motion initiation
        self.linear = linear
        self.angular = angular
        self.pint = primary_interface(robotname)
        self.pint.update_body_vel(linear, angular)
        # sensor interrupt loop code
    def DefaultMotion(self):
        self.pint.update_body_vel(self.linear, self.angular)


if __name__ == "__main__":
    rospy.init_node('DriveTest', anonymous=True)
    miro1 = SecondaryInterface('rob01', 1, 1)
    rospy.spin()
