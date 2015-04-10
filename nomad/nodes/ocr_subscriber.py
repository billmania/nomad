#!/usr/bin/env python

import roslib; roslib.load_manifest('nomad')
import rospy

from std_msgs.msg import String

class OCRSubscriber():

    def ocrCallback(self,msg):
        print msg.data


    def __init__(self):

        # get most recent twist on cmd_vel
        self.cmd_vel1 = rospy.Subscriber('/ocr_text', String, self.ocrCallback)

if __name__ == '__main__':

    rospy.init_node('OCRSubscriber')
    ocr = OCRSubscriber()
    rospy.spin()
