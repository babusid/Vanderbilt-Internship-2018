from interfaces import *

# A secondary interface built on top of interfaces.py
# designed to allow a default state other than standing still
# and to create simpler methods for certain actions
# created by Sidharth Babu 7/25/18

class secondary_interface:

    class secondary_interface:

        def __init__(self, robot_name, deflinear, defangular):
            self.pint = primary_interface(robot_name)
            self.pint.update_body_vel(deflinear, defangular)
