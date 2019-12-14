#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from maze.msg import Maze
import astar
from scipy.spatial.transform import Rotation as R
import sys
from fixed_path_controller import Controller

class DynamicController(Controller):
    """
    This controller will be able to handle the changing maze, 
    with the assuption that the total size of the maze is not changed.
    """


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

        # only update the state when the message is valid
        if self.validate(maze, 100):
            # start = self.localize(maze.turtlebot_pos, maze.map_shape, maze.seg_img_shape)
            start = self.localize_turtlebot(maze.turtlebot_pos, maze.map_shape, maze.seg_img_shape)
        # else:
        #     print("Invalid maze, state update is estimated")
        #     self.estimate_state()
        #     return

            self.state_pixel = np.asarray(maze.turtlebot_pos)

            # if maze is meaningful, update every attribute
            if maze.success:
                # update the state
                x, y = np.asarray(start)
                r = maze.transform.rotation
                rotation = R.from_quat([r.x,r.y,r.z,r.w]).as_euler('xyz', degrees=False)
                angle = rotation[2]
                self.state = (x, y, angle)

                self.goal_pixel = np.asarray(maze.goal_pos)
            
                goal = self.localize(maze.goal_pos, maze.map_shape, maze.seg_img_shape)
                self.grid = np.asarray(maze.occupancy_grid).reshape(maze.map_shape[0], maze.map_shape[1])
                planner = astar.Planner(self.grid, start, goal)
                self.path = np.asarray(planner.findPath())
                if len(self.path) >= 2:
                    self.end = self.path[1]
                    self.scale = np.asarray(maze.seg_img_shape) / maze.map_shape
                    self.dist_tolerance = np.min(self.scale) / self.dist_factor
                    if tuple(start) in self.path:
                        self.is_turning = True
                    else:
                        self.is_turning = False


       
        
        # print('========================================')
        # print('End: ', self.end)
        # print('State: ', self.state)
        # print('State pixel: ', self.state_pixel)
        # print('End pixel: ', np.flip((self.end + [0.5, 0.5])* self.scale).astype(int))
        # print('Goal pixel: ', self.goal_pixel)
        # print('Grid: ', self.grid)
        # print('Path: ', self.path)
        # print('current motion: from ', self.state, ' to ', self.end)
        # print('Scale: ', self.scale)

    def localize_turtlebot(self, pos, map_shape, img_shape):
        """
        localize the position of turtlebot
        only use the grid position if the distance from the center of the turtlebot to the center of the rid
        is within the distance tolerance
        """

        temp_pos = (np.flip(pos).astype(float) / img_shape) * map_shape
        
        if self.end is None or self.distance_to_end(pos) < self.dist_tolerance:
            return temp_pos.astype(int)
        else:
            return np.array([self.state[0], self.state[1]])

    

    def distance_to_end(self, pos):
        """
        compute the distance from the pos to self.end
        """

        end_pixel = np.flip((self.end + [0.5, 0.5])* self.scale).astype(int)
        return np.linalg.norm(end_pixel - pos)
            


    def control(self):
        """
        The main control function
        """

        # self.message_rate = 10
        self.Kz /= 2
        self.Kl /= 2
        # self.angle_tolerance = 0.1
        self.dist_factor = 4
        self.goal_tolerance = 70

        rospy.init_node("controller", anonymous=True)
        r = rospy.Rate(self.message_rate)
        # self.linear_factor = 0.01

        while not rospy.is_shutdown() and not self.is_goal(self.goal_tolerance):
            rospy.Subscriber('/maze/grid', Maze, self.update)
            desired = self.compute_desired_angle()
            self.linear_factor = 0.01

            if desired is not None:
                current_path = self.path

                # desired angle is not changed until turtlebot reaches the next grid position and path changes
                while not rospy.is_shutdown() and not self.is_goal(self.goal_tolerance) and np.array_equal(current_path, self.path):
                    angle_diff =  desired - self.state[2]
                    dist_diff = self.distance()
                    # print('Path length', path_length)
                    print("path: ", self.path)
                    # keep the angle_diff in the range of -pi to pi
                    if angle_diff > np.pi:
                        angle_diff -= 2 * np.pi
                    if angle_diff < -np.pi:
                        angle_diff += 2 * np.pi

                    print("Angle difference: ", angle_diff)
                    print('current motion: from ', self.state, ' to ', self.end)
                    cmd = Twist()

                    # pure rotation at turning points
                    # if abs(angle_diff) > self.angle_tolerance and hasattr(self, 'is_turning') and self.is_turning:
                    if abs(angle_diff) > self.angle_tolerance:
                        self.linear = 0.0
                        cmd.angular.z = self.angular = self.Kz * angle_diff
                        # if hasattr(self, 'is_turning') and self.is_turning: 
                        #     self.linear_factor = 0.01
                      
                    # rotation and translation 
                    elif dist_diff > self.dist_tolerance:
                        cmd.linear.x = self.linear = self.Kl * dist_diff  * self.linear_factor
                        cmd.angular.z = angle_diff
                        self.angular = 0.0
                        if self.linear_factor < 1:
                            self.linear_factor += 0.01
                    # elif len(self.path) < path_length:
                    #     print("should start turning now")
                    #     break
                        
                    self.pub.publish(cmd)
                    # print(cmd)
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
    controller = DynamicController(sys.argv[1])
    controller.control()