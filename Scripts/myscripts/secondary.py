from interfaces import*


class SecondaryInterface:

    def __init__(self, robotname, linear, angular):
        # default motion initiation
        self.linear = linear
        self.angular = angular
        self.pint = primary_interface(robotname)
        self.pint.update_body_vel(linear, angular)
        # sensor interrupt loop start

    # def default(self):
      #  self.pint.update_body_vel(self.linear, self.angular)

    def interrupt(self):
        if self.pint.touch_head:
            self.pint.stop_moving()
        if self.pint.touch_body:
            self.pint.stop_moving()
        else:
            self.pint.update_body_vel(self.linear, self.angular)

