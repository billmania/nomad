#!/usr/bin/env python

"""

obstacle detector and collision avoider

- monitors the LaserScan messages, watching for ranges which are
  less than a threshold. Adjusts the rovers direction to avoid
  collision.

"""

import random, math
import roslib; roslib.load_manifest('nomad')
import rospy

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class CollisionDetector():

    def __init__(self):
        self.ackermannMessage =  AckermannDriveStamped()
        self.avoidAckermannMessage = AckermannDriveStamped()

        self.avoidAckermannMessage.drive.speed = 0.0
        self.avoidAckermannMessage.drive.steering_angle = 0.0
        self.minimum_range_to_obstacle = 1.0 # meters
        self.backup_speed = 0.3 # meters per second
        self.turn_angle = 0.7 # radians
        self.already_turning = False
        self.turn_direction = random.choice([-1, 1])
        #
        # after the ranges have been sorted into ascending order, which
        # is the first one to actually examine.
        #
        self.range_to_examine = 2
        rospy.loginfo("Initializing collision_detector node with obstacle distance threshold %0.1f" % (self.minimum_range_to_obstacle))

        self.filtered_ackermann = rospy.Publisher('filtered_ackermann_cmd', AckermannDriveStamped, queue_size = 10)
        self.filtered_ackermann.publish(self.ackermannMessage)

        #
        # subscribe to the ackermann commands and the scan data
        #
        self.ackermannSubscriber = rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, self.handleAckermannMessage)
        self.scanSubscriber = rospy.Subscriber('scan', LaserScan, self.scanCallback)

    def turnDirection(self):
        """turnDirection() - determine which way to turn

        If the rover isn't already turning, randomly choose a direction. Otherwise,
        continue to turn in the same direction.
        """

        if not self.already_turning:
            self.already_turning = True
            self.turn_direction = random.choice([-1, 1])
            rospy.logdebug('Setting turn direction to %d' % self.turn_direction)

        return self.turn_direction

    def handleAckermannMessage(self, ackermannMessage):
        """handleAckermannMessage() - Just save the most recent Ackermann message

        """

        rospy.logdebug("speed: %f, turn: %f" % (
            ackermannMessage.drive.speed,
            ackermannMessage.drive.steering_angle
            ))

        self.ackermannMessage = ackermannMessage

        return

    def scanCallback(self, msg):

        if self.ackermannMessage.drive.speed == 0 and self.ackermannMessage.drive.steering_angle == 0:
            self.filtered_ackermann.publish(self.ackermannMessage)
            return

        #
        # Find the laser scan ranges between -pi/4 and pi/4.
        #
        number_of_ranges = len(msg.ranges)
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        range_min = msg.range_min
        angle_increment = msg.angle_increment
        rospy.logdebug('ranges: %d, minAngle: %f, maxAngle: %f, increment: %f, range_min: %f' % (
            number_of_ranges,
            min_angle,
            max_angle,
            angle_increment,
            range_min))
        calculated_ranges = (abs(min_angle) + max_angle) / angle_increment + 1
        xRay = int(abs(min_angle) / angle_increment)
        oneEighth = int(math.pi / 4 / angle_increment)
        leftQuarter = xRay - oneEighth
        rightQuarter = xRay + oneEighth
        rospy.logdebug('calculated ranges: %d, left: %d, right: %d' % (
            calculated_ranges,
            leftQuarter,
            rightQuarter))

        left_side = sorted(msg.ranges[0:leftQuarter])
        right_side = sorted(msg.ranges[rightQuarter:])
        front = sorted(msg.ranges[leftQuarter:rightQuarter])

        something_on_the_left = range_min < left_side[self.range_to_examine] <= self.minimum_range_to_obstacle
        something_on_the_right = range_min < right_side[self.range_to_examine] <= self.minimum_range_to_obstacle

        if range_min < front[self.range_to_examine] <= self.minimum_range_to_obstacle:
            # something is directly in front of the rover
            rospy.logdebug('Something %f meters in front' % front[self.range_to_examine])
            self.avoidAckermannMessage.drive.speed = 0.0

            if something_on_the_left and something_on_the_right:
                rospy.logdebug(' and on both sides')
                self.avoidAckermannMessage.drive.speed = -1 * self.backup_speed
                self.avoidAckermannMessage.drive.steering_angle = self.turn_angle * self.turnDirection()
                self.filtered_ackermann.publish(self.avoidAckermannMessage)

            elif something_on_the_left and not something_on_the_right:
                rospy.logdebug(' and on the left')
                self.avoidAckermannMessage.drive.steering_angle = self.turn_angle
                self.filtered_ackermann.publish(self.avoidAckermannMessage)

            elif not something_on_the_left and something_on_the_right:
                rospy.logdebug(' and on the right')
                self.avoidAckermannMessage.drive.steering_angle = -(self.turn_angle)
                self.filtered_ackermann.publish(self.avoidAckermannMessage)

            else:
                self.avoidAckermannMessage.drive.steering_angle = self.turn_angle * self.turnDirection()
                self.filtered_ackermann.publish(self.avoidAckermannMessage)

        else:
            if something_on_the_left and not something_on_the_right:
                rospy.logdebug(' Something on the left')
                self.ackermannMessage.drive.steering_angle = self.ackermannMessage.drive.steering_angle + (self.turn_angle / 4)
                self.filtered_ackermann.publish(self.ackermannMessage)

            elif not something_on_the_left and something_on_the_right:
                rospy.logdebug(' Something on the right')
                self.ackermannMessage.drive.steering_angle = self.ackermannMessage.drive.steering_angle - (self.turn_angle / 4)
                self.filtered_ackermann.publish(self.ackermannMessage)

            else:
                rospy.logdebug('Clear sailing!')
                self.already_turning = False
                self.filtered_ackermann.publish(self.ackermannMessage)

        return

if __name__ == '__main__':

    rospy.init_node('obstacle_detect_and_avoid', log_level = rospy.DEBUG)
    collision_detector = CollisionDetector()
    rospy.spin()

