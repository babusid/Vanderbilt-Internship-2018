def pet_pat(self):
    self.body_config_speed = [5, 5, 5, 5]
    if self.touch_body:

        # find average value of body touch
        try:
            # avg body touch algorithm
            self.value = (self.touch_body[0] + 2.0 * self.touch_body[1] + 3.0 *
                          self.touch_body[2] + 4.0 * self.touch_body[3]) / (
                             numpy.sum(self.touch_body))
        except ZeroDivisionError:
            self.value = 0

            if self.value != 0:
                self.body_config = [0, 0, 0, 1]
                time.sleep(.5)
                self.body_config = [0, 0, 0, 0]
            else:
                self.body_config = [0, 0, .2, 0]
                time.sleep(.25)
                self.body_config = [0, 0, -2, 0]
                time.sleep(.25)
                self.body_config = [0, 0, 0, 0]
