#!/usr/bin/env python

"""

drive_manually

reads one character manual drive commands from stdin, creates an appropriate
AckermannDriveStamped message and publishes it to the /ackermann_cmd topic.
this program is intended to be used to manually drive the mower.

listens to:

publishes to:
 /ackermann_cmd
"""

import roslib; roslib.load_manifest('nomad')
import rospy

from math import pi

from ackermann_msgs.msg import AckermannDriveStamped

import os
import sys
import termios
import fcntl

speedIncrement = 0.05
steerIncrement = pi / 32.0
adjuster = 0.9
FULL_STOP = ' '
keyToRate = {
    '7' : (speedIncrement, 0.0, 'AHEAD FASTER'),
    'u' : (-speedIncrement, 0.0, 'AHEAD SLOWER'),
    FULL_STOP : (0.0, 0.0, 'FULL STOP'),
    'j' : (speedIncrement, 0.0, 'REVERSE SLOWER'),
    'm' : (-speedIncrement, 0.0, 'REVERSE FASTER'),
    'y' : (0.0, steerIncrement, 'TURN LEFT'),
    'i' : (0.0, -steerIncrement, 'TURN RIGHT'),
}

def driveLoop():
    """
    driveLoop() - a loop which reads from stdin and publishes a
        AckermannDriveStamped message, ad infinitum
    """

    drivePublisher = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size = 1)

    ackermannMessage = AckermannDriveStamped()

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
                drivePublisher.publish(ackermannMessage)
                rate.sleep()

        try:
            # build the AckermannDriveStamped message
            if c == FULL_STOP:
                ackermannMessage.drive.speed = 0.0
                ackermannMessage.drive.steering_angle = 0.0
            else:
                ackermannMessage.drive.speed += keyToRate[c][0] * adjuster
                ackermannMessage.drive.steering_angle += keyToRate[c][1] * adjuster
            print keyToRate[c][2]

            # publish the AckermannDriveStamped message
            drivePublisher.publish(ackermannMessage)

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
