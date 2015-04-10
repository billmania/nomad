#!/usr/bin/env python

"""

obstacle detector and collision avoider

- monitors the LaserScan messages, watching for ranges which are
  less than a threshold. Adjusts the rovers direction to avoid
  collision.

"""

import random
import roslib; roslib.load_manifest('nomad')
import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist


class CollisionDetector():

    def __init__(self):
        self.twistMessage =  Twist()
        self.avoidTwistMessage = Twist()

        self.avoidTwistMessage.linear.x = 0.0
        self.avoidTwistMessage.angular.z = 0.0
        self.minimum_range_to_obstacle = 0.7 # meters
        self.backup_speed = 0.3 # meters per second
        self.turn_speed = 0.7 # radians per second
        self.already_turning = False
        self.turn_direction = random.choice([-1, 1])
        #
        # after the ranges have been sorted into ascending order, which
        # is the first one to actually examine.
        #
        self.range_to_examine = 2
        rospy.loginfo("Initializing collision_detector node with obstacle distance threshold %0.1f" % (self.minimum_range_to_obstacle))

        self.filtered_cmd_vel1 = rospy.Publisher('filtered_cmd_vel',Twist, queue_size = 10)
        self.filtered_cmd_vel1.publish(self.twistMessage)

        # get most recent twist on cmd_vel
        self.cmd_vel1 = rospy.Subscriber('cmd_vel', Twist, self.handleTwistMessage)

        # callback on laser
        self.callback = rospy.Subscriber('scan', LaserScan, self.laser_callback)

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

    def handleTwistMessage(self,twistMessage):
        """handleTwistMessage() - Just save the most recent Twist message

        """

        rospy.logdebug("translationX: %f, rotationZ: %f" % (
            twistMessage.linear.x,
            twistMessage.angular.z
            ))

        self.twistMessage = twistMessage

        return

    def laser_callback(self, msg):

        if self.twistMessage.linear.x == 0 and self.twistMessage.angular.y == 0:
            self.filtered_cmd_vel1.publish(self.twistMessage)
            return

        #
        # Divide the scan data into three sections: the left quarter, the middle half
        # and the right quarter. This logic assumes the LiDAR has a Field of View of
        # 180 degrees. From the front section, extract the small section which is
        # most directly in front of the rover.
        #
        number_of_ranges = len(msg.ranges)
        end_of_left_side = number_of_ranges / 4
        begin_of_right_side = 3 * number_of_ranges / 4

        left_side = sorted(msg.ranges[0:end_of_left_side])
        right_side = sorted(msg.ranges[begin_of_right_side:])
        front = sorted(msg.ranges[end_of_left_side:begin_of_right_side])
        far_front = sorted(msg.ranges[7 * number_of_ranges / 16:9 * number_of_ranges / 16])

        something_on_the_left = left_side[self.range_to_examine] < self.minimum_range_to_obstacle
        something_on_the_right = right_side[self.range_to_examine] < self.minimum_range_to_obstacle

        if front[self.range_to_examine] < self.minimum_range_to_obstacle:
            # something is directly in front of the rover
            rospy.logdebug('Something in front')
            self.avoidTwistMessage.linear.x = 0.0

            if something_on_the_left and something_on_the_right:
                rospy.logdebug(' and on both sides')
                self.avoidTwistMessage.linear.x = -1 * self.backup_speed
                self.avoidTwistMessage.angular.z = self.turn_speed * self.turnDirection()
                self.filtered_cmd_vel1.publish(self.avoidTwistMessage)

            elif something_on_the_left and not something_on_the_right:
                rospy.logdebug(' and on the left')
                self.avoidTwistMessage.angular.z = self.turn_speed
                self.filtered_cmd_vel1.publish(self.avoidTwistMessage)

            elif not something_on_the_left and something_on_the_right:
                rospy.logdebug(' and on the right')
                self.avoidTwistMessage.angular.z = -(self.turn_speed)
                self.filtered_cmd_vel1.publish(self.avoidTwistMessage)

            else:
                self.avoidTwistMessage.angular.z = self.turn_speed * self.turnDirection()
                self.filtered_cmd_vel1.publish(self.avoidTwistMessage)

        else:
            if far_front[self.range_to_examine] < (self.minimum_range_to_obstacle * 1.5):
                # the rover is approaching something in the distance
                rospy.logdebug('Something approaching')
                self.twistMessage.linear.x = self.twistMessage.linear.x / 2.0
                self.filtered_cmd_vel1.publish(self.twistMessage)

            if something_on_the_left and not something_on_the_right:
                rospy.logdebug(' Something on the left')
                self.twistMessage.angular.z = self.twistMessage.angular.z + (self.turn_speed / 4)
                self.filtered_cmd_vel1.publish(self.twistMessage)

            elif not something_on_the_left and something_on_the_right:
                rospy.logdebug(' Something on the right')
                self.twistMessage.angular.z = self.twistMessage.angular.z - (self.turn_speed / 4)
                self.filtered_cmd_vel1.publish(self.twistMessage)

            else:
                rospy.logdebug('Clear sailing!')
                self.already_turning = False
                self.filtered_cmd_vel1.publish(self.twistMessage)

        return

if __name__ == '__main__':

    rospy.init_node('obstacle_detect_and_avoid', log_level = rospy.DEBUG)
    collision_detector = CollisionDetector()
    rospy.spin()

