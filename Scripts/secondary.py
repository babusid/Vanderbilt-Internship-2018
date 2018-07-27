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
        pass
