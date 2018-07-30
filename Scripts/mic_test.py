import rospy
from std_msgs.msg import String
from miro_msgs.msg import platform_mics, platform_sensors
from array import array
from interfaces import primary_interface
import operator
import time
import numpy


class mic_test:

    def __init__(self):
        robot_name = "rob01"
        root = "/miro/" + str(robot_name) + "/platform"
        self.data_in = rospy.Subscriber(root + "/mics", platform_mics, self.data_in, queue_size=1)
        self.primary_int = primary_interface(robot_name)

    def data_in(self, data):

        left_mic = []
        right_mic = []
        even_cnt = 1
        linear = 0
        angular = 0

        # reverse interleaving to seperate left and right ear data and store in individual lists
        for x in range(4000):
            if even_cnt == 1:

                left_mic.append(data.data[x])
                even_cnt = 0

            elif even_cnt == 0:

                right_mic.append(data.data[x])
                even_cnt = 1

        # find maximum cross correlation of mics
        if left_mic:
            max_mic = max(left_mic)
            max_mic_index = left_mic.index(max_mic)
            left_mic[max_mic_index - 50:max_mic_index + 50]
            right_mic[max_mic_index - 50:max_mic_index + 50]
            corr = numpy.ndarray.tolist(numpy.correlate(left_mic, right_mic, "same"))
            max_corr = max(corr[994:1006])
            max_index = corr.index(max_corr)

        # if sound is sufficiently loud, move towards sound source
        if max_corr > 400000:

            if max_index >= 994 and max_index <= 995:
                linear = .1
                angular = 1
                self.primary_int.update_body_vel(linear, angular)
                time.sleep(1.4)
            elif max_index >= 996 and max_index <= 997:
                linear = .1
                angular = 1
                self.primary_int.update_body_vel(linear, angular)
                time.sleep(1)
            elif max_index >= 998 and max_index <= 999:
                linear = .1
                angular = 1
                self.primary_int.update_body_vel(linear, angular)
                time.sleep(.6)
            elif max_index == 1000:
                linear = .1
                angular = 0
                self.primary_int.update_body_vel(linear, angular)
            elif max_index >= 1001 and max_index <= 1002:
                linear = .1
                angular = -1
                self.primary_int.update_body_vel(linear, angular)
                time.sleep(.6)
            elif max_index >= 1003 and max_index <= 1004:
                linear = .1
                angular = -1
                self.primary_int.update_body_vel(linear, angular)
                time.sleep(1)
            elif max_index >= 1005 and max_index <= 1006:
                linear = .1
                angular = -1
                self.primary_int.update_body_vel(linear, angular)
                time.sleep(1.4)
            angular = 0
            self.primary_int.update_body_vel(linear, angular)

        if self.primary_int.sonar_range < .3 and self.primary_int.sonar_range != 0.0:
            linear = 0
            self.primary_int.update_body_vel(linear, angular)


if __name__ == "__main__":
    rospy.init_node("mic_test", anonymous=True)
    behavior = mic_test()
    rospy.spin()
