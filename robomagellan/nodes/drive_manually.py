#!/usr/bin/env python

"""

drive_manually

reads one character manual drive commands from stdin, creates
an appropriate Twist message and publishes it to the /cmd_vel
topic. this program is intended to be used to manually drive
the rover.

listens to:

publishes to:
 /cmd_vel
"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

import time

from geometry_msgs.msg import Twist

import os
import sys    
import termios
import fcntl

translateRate = 0.7
rotateRate = 0.6
keyToRate = {
    '7' : (translateRate, 0.0, 'AHEAD FULL'),
    'u' : (translateRate / 2, 0.0, 'AHEAD HALF'),
    ' ' : (0.0, 0.0, 'FULL STOP'),
    'j' : (-translateRate / 2, 0.0, 'REVERSE HALF'),
    'm' : (-translateRate, 0.0, 'REVERSE FULL'),
    'y' : (translateRate, rotateRate, 'AHEAD LEFT'),
    'i' : (translateRate, -rotateRate, 'AHEAD RIGHT'),
    'h' : (-translateRate, -rotateRate, 'REVERSE LEFT'),
    'k' : (-translateRate, rotateRate, 'REVERSE RIGHT'),
    'o' : (0.0, rotateRate * 2, 'ROTATE IN PLACE TO THE LEFT'),
    'p' : (0.0, -rotateRate * 2, 'ROTATE IN PLACE TO THE RIGHT')
}
adjuster = 1.0

def driveLoop():
    """
    driveLoop() - a loop which reads from stdin and publishes a
        Twist message, ad infinitum
    """

    drivePublisher = rospy.Publisher('cmd_vel', Twist)

    twistMessage = Twist()

    fd = sys.stdin.fileno()
    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)
    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():

        # read one character
        while True:            
            try:
              c = sys.stdin.read(1)
              break

            except IOError:
                # in simulation, we need to publish every loop
                drivePublisher.publish(twistMessage)
                time.sleep(0.25)

        try:
            # build the Twist message
            twistMessage.linear.x = keyToRate[c][0] * adjuster
            twistMessage.angular.z = keyToRate[c][1] * adjuster
            print keyToRate[c][2]

            # publish the Twist message
            drivePublisher.publish(twistMessage)

        except:
            if c == 'q':
                break

        rate.sleep()

    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

if __name__ == '__main__':
    rospy.init_node('drive_manually')
    rospy.loginfo("drive_manually started")

    driveLoop()

    sys.exit(0)
