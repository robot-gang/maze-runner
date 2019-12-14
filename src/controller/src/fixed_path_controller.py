#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from maze.msg import Maze
import astar
from scipy.spatial.transform import Rotation as R
import sys

class Controller:
    """
    The angle_tolerance, Kz, dist_tolerance, and Kl are coupled together.
    When tuning these parameters, we need to make sure they are in the appropriate ranges.
    """
    angle_tolerance = 0.15
    dist_factor = 3
    Kl = 0.0007     # angular velocity factor
    Kz = 0.7      # linear velocity factor
    message_rate = 30
    goal_tolerance = 150


    def __init__(self, bot_color):
        """
        end: is each turning point on the path
        state: contains the current position and angle, (x, y, angle)
        grid: is the 2D array of the maze
        path: is the result of the path planning phase
        """

        self.state = None
        self.state_pixel = None
        self.end = None
        self.goal_pixel = None
        self.grid = np.empty(1)
        self.path = []
        self.scale = np.zeros(2)

        # store the current linear and angular velocity, they may be usded to estimate the position of turtlebot
        self.linear = 0.0
        self.angular = 0.0

        self.pub = rospy.Publisher('/' + bot_color + '/mobile_base/commands/velocity', Twist, queue_size=10)


    def compute_desired_angle(self):
        """
        Compute the angle between current state and the current end wrt. goal frame (fixed frame)
        """

        if self.end is not None and any(self.state):
            end_pixel = np.flip((self.end + [0.5, 0.5])* self.scale).astype(int)
            dx, dy = end_pixel - self.state_pixel
            return np.arctan2(dx, dy)


    def estimate_state(self):
        """
        when the message received form the subscriber is not valid, use this method to estimate the state
        only update if the scale is not zero and the previous state is not None
        """
        if self.state is not None and all(self.scale != 0):
            if self.linear:
                translation = self.linear / self.message_rate 
                self.state_pixel += (translation * np.array([np.cos(self.state[2]), np.sin(self.state[2])])).astype(int)
                x, y = (np.flip(self.state_pixel) / self.scale).astype(int)
                self.state = (x, y, self.state[2])

            if self.angular:
                rotation = self.angular / self.message_rate
                self.state = (self.state[0], self.state[1], self.state[2] + rotation) 
       

    def is_goal(self, tolerance):
        """
        check if the turtlebot is close to goal 
        """
        
        # if self.goal_pixel is not None:
            # print("Distance to goal: ", np.linalg.norm(self.goal_pixel - self.state_pixel))
        return (self.goal_pixel is not None) and np.linalg.norm(self.goal_pixel - self.state_pixel) < tolerance 


    def validate(self, maze, tolerance):
        """
        Check if the input maze message is valid, by comparing the tutlebot_pos to the previous position stored in self.state_pixel
        """
        pos = maze.turtlebot_pos

        # all negative positions are invalid
        if any(np.asarray(pos) < 0):
            return False
        elif self.state_pixel is None:
            return True
        # if the position does not change dramatically, then it is valid
        elif np.sum(abs(self.state_pixel - pos)) < tolerance:
            return True
        else:
            return False


    def update(self, maze):
        """
        Input:
        - maze: the message received from vision
            + transformation
            + grid
            + map_shape
            + turtlebot_pos: in segmentation pixel
            + goal_pos: in segmentation pixel
            + image_shape: segmentation image

        Do
        - Update instance attributes
            + grid, path: only once
            + state: every message
        """

        if self.validate(maze, 100):
            start = self.localize(maze.turtlebot_pos, maze.map_shape, maze.seg_img_shape)
        else:
            self.estimate_state()
            return

        # Update the state (x, y, angle) of the turtlebot
        x, y = np.asarray(start)
        r = maze.transform.rotation
        rotation = R.from_quat([r.x,r.y,r.z,r.w]).as_euler('xyz', degrees=False)
        angle = rotation[2]

        self.state = (x, y, angle)
        self.state_pixel = np.asarray(maze.turtlebot_pos)
        self.goal_pixel = np.asarray(maze.goal_pos)

        # Update grid, path and end when needed
        if self.end is None:
            goal = self.localize(maze.goal_pos, maze.map_shape, maze.seg_img_shape)
            self.grid = np.asarray(maze.occupancy_grid).reshape(maze.map_shape[0], maze.map_shape[1])
            planner = astar.Planner(self.grid, start, goal)
            self.path = np.asarray(planner.findPath())
            if len(self.path) >= 2:
                self.end = self.path[1]
                self.scale = np.asarray(maze.seg_img_shape) / maze.map_shape
                self.dist_tolerance = np.min(self.scale) / self.dist_factor
     

    def distance(self):
        """
        return the euclidean distance between the CURRENT state and the END
        """

        end_pixel = np.flip((self.end + [0.5, 0.5])* self.scale).astype(int)
        return np.linalg.norm(end_pixel - self.state_pixel)


    def localize(self, pos, map_size, img_shape):
        """
        compute the grid x, y coordinate from pixels
        """
        return ((np.flip(pos).astype(float) / img_shape) * map_size).astype(int)


    def control(self):
        """
        The main control function
        """

        rospy.init_node("controller", anonymous=True)
        r = rospy.Rate(self.message_rate)

        while not rospy.is_shutdown():
            rospy.Subscriber('/maze/grid', Maze, self.update)
            desired = self.compute_desired_angle()

            if desired is not None and len(self.path) > 1:
                linear_factor = 0.01

                while not rospy.is_shutdown() and not self.is_goal(self.goal_tolerance):
                    angle_diff =  desired - self.state[2]
                    dist_diff = self.distance()


                    # keep the angle_diff in the range of -pi to pi
                    if angle_diff > np.pi:
                        angle_diff -= 2 * np.pi
                    if angle_diff < -np.pi:
                        angle_diff += 2 * np.pi

                    cmd = Twist()

                    # pure rotation at turning points
                    if abs(angle_diff) > self.angle_tolerance:
                        cmd.angular.z = self.angular = self.Kz * angle_diff
                        linear_factor = 0.01
                        self.linear = 0.0
                    
                    # rotation and translation 
                    elif dist_diff > self.dist_tolerance:
                        cmd.linear.x = self.linear = self.Kl * dist_diff * linear_factor
                        cmd.angular.z = angle_diff
                        self.angular = 0.0
                        if linear_factor < 1:
                            linear_factor += 0.1
                    
                    # reach the current end, update path and the end
                    else: 
                        self.path = self.path[1:]
                        if len(self.path) > 1:
                            self.end = self.path[1]
                        break
                    self.pub.publish(cmd)
                    r.sleep()

            # keep moving turtlebot using the previous twist for certain times
            if self.is_goal(self.goal_tolerance):
                for _ in range(60):
                    self.pub.publish(cmd)
                    r.sleep() 
                print('Goal is reached!')
                break

            r.sleep()


if __name__ == '__main__':
    controller = Controller(sys.argv[1])
    controller.control()
