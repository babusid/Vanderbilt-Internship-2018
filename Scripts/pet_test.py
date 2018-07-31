import rospy
from std_msgs.msg import String
from miro_msgs.msg import platform_mics, platform_sensors
from array import array
from interfaces import primary_interface
import time
import numpy
from sound_test import sound_test


class pet_test:

    def __init__(self):

        robot_name = "rob01"
        self.primary_int = primary_interface(robot_name)

    def data_in(self):

        while not rospy.is_shutdown():

            start_time = rospy.get_rostime()
            self.old_value = 0.0
            flag_pet = False
            flag_pat = False
            self.primary_int.body_config_speed = [5, 5, 5, 5]

            while not rospy.is_shutdown():

                if self.primary_int.touch_body:

                    # find average value of body touch
                    try:
                        self.value = (self.primary_int.touch_body[0] + 2.0 * self.primary_int.touch_body[1] + 3.0 *
                                      self.primary_int.touch_body[2] + 4.0 * self.primary_int.touch_body[3]) / (
                                         numpy.sum(self.primary_int.touch_body))
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
                            print
                            flag_pet, flag_pat
                            start_time = rospy.get_rostime()
                        # one flag activated
                        elif flag_pet == True:

                            # if next event occurs within 1 sec, initiates response
                            if self.value != 0 and (rospy.get_rostime() - start_time).to_sec() < 1.0:
                                self.pet_detect()
                                flag_pet = False
                                time.sleep(.5)
                            else:
                                flag_pet = False
                        elif flag_pat == True:

                            if (rospy.get_rostime() - start_time).to_sec() < 1.0:
                                self.pat_detect()
                                flag_pat = False
                                time.sleep(.5)
                            else:
                                flag_pat = False
                    self.old_value = self.value

    def pet_detect(self):
        print('pet')

    def pat_detect(self):
        print("pat")

if __name__ == "__main__":
    rospy.init_node("mic_test", anonymous=True)
    behavior = pet_test()
    behavior.data_in()
