from .interfaces import*

class SecondaryInterface:

    def __init__(self, robotname, linear, angular):
        # default motion initiation
        self.linear = linear
        self.angular = angular
        self.pint = primary_interface(robotname)
        self.pint.update_body_vel(linear, angular)
        # sensor interrupt loop start

    def default(self) :
        self.pint.update_body_vel(self.linear, self.angular)

    def interrupt(self):
        if self.pint.touch_head:
            pass
        if self.pint.touch_body:
            pass
        else
            self.default()


