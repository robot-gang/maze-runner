#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

def controller(path, orientation):
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
    # tfBuffer = tf2_ros.Buffer()
    # tfListerner = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(10)
    K1 = 0.3
    K2 = 1

    while not rospy.is_shutdown():

        #trans = tfBuffer.lookup_transform()
        while len(path) > 1:
            start = path.pop(0)
            end = path[0]
            #print(start, end)
            rotate, orientation = cmd_rotate(start, end, orientation)
            #pub.publish(rotate)
            publisher('angular', orientation)
            r.sleep()
            #translate = cmd_translate(start, end)
            #pub.publish(translate)
            publisher('linear', 0)
            r.sleep()

NUM_PER_TILE = 14

def publisher(option, angle):
    """num is the number of Twists that will be published"""

    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
    r = rospy.Rate(5)
    target_speed = 0.2
    target_turn = 1
    control_speed = 0
    control_turn = 0
    for i in range(NUM_PER_TILE):
        twist = Twist()
        twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
        pub.publish(twist)
        if option == 'linear':
            print('linear')
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.022 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.022 )
            else:
                control_speed = target_speed
        else:
            print('angular')
            if angle > 0:
                if target_turn > control_turn:
                    control_turn = min( target_turn, control_turn + 0.13 )
                elif target_turn < control_turn:
                    control_turn = max( target_turn, control_turn - 0.13 )
                else:
                    control_turn = target_turn
            elif angle < 0:
                target_turn *= -1
                if target_turn < control_turn:
                    control_turn = max( target_turn, control_turn - 0.13 )
                elif target_turn > control_turn:
                    control_turn = min( target_turn, control_turn + 0.13 )
                else:
                    control_turn = target_turn
            else:
                break
        r.sleep()

def cmd_rotate(start, end, orientation):
    """
    assume that the orientation of turtleBot at start position is known,
    represented by angles, 0, 45, 90, 135, 180
    counterclockwise is positive
    """
    # pure rotation
    cmd = Twist()
    deltaX = end[0] - start[0]
    deltaY = end[1] - start[1]
    curr_orientation = np.arctan2(deltaY, deltaX)
    angle = curr_orientation - orientation
    if angle > np.pi:
        angle -= 2 * np.pi
    elif angle < - np.pi:
        angle += 2 * np.pi
    cmd.angular.z = angle

    return cmd, curr_orientation

def cmd_translate(start, end):
    cmd = Twist()
    cmd.linear.x = end[0] - start[0]
    cmd.linear.y = end[1] - start[1]

    return cmd


if __name__ == '__main__':
    rospy.init_node('controller',  anonymous=True)
    path = [(0, 0), (1, 0), (2, 1), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4), (3, 5), (2, 5), (1, 5), (1, 4), (0, 4)]
    controller(path, 0)
