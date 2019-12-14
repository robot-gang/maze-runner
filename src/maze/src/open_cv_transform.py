#!/usr/bin/env python

"""
RUNNING INSTRUCTIONS:

rosrun maze open_cv_transform.py 3 4 green red ar_marker_0 ar_marker_1

(rosrun maze open_cv_transform.py grid_shape[0] grid_shape[1] background_color turtlebot_color turtlebot_frame goal_frame)


NOTE: CHANGED SOME CODE TO WORK WITH REALSENSE CAMERA
ONLY EFFECTS DECLARATION OF MAZE PROCESS OBJECT IN MAIN FUNCTION AND SOME THRESHOLDING FOR GREEN IN UTILS
"""


import cv2
import matplotlib.pyplot as plt
import message_filters
import numpy as np
import rospy
import ros_numpy
import sys
import tf2_ros

from ar_track_alvar_msgs.msg import AlvarMarkers
from collections import deque
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform
from maze.msg import Maze
from rospy.numpy_msg import numpy_msg
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from utils import *

background_color = sys.argv[3]
turtlebot_color = sys.argv[4]

# goes turtlebot frame, then goal frame
frame_id = int(sys.argv[5])
map_size = map(int, sys.argv[1:3])

prev_corners = None 
class NoiseTracker:
    def __init__(self, tolerance=30, queue_size=7):
        self.prev_queue = deque([], queue_size)
        self.tolerance = tolerance
        self.allow_update = np.ones(1)        

def update_turtlebot_pos(turtlebot_pos, tracker):
    if tracker is not None and len(tracker.prev_queue) > 0:
        median = np.median(np.asarray(tracker.prev_queue), axis=0)
        # print(median)
        tracker.prev_queue.appendleft(turtlebot_pos)
        if abs(turtlebot_pos[0]-median[0]) > tracker.tolerance and abs(turtlebot_pos[1]-median[1]) > tracker.tolerance:
            return median
    tracker.prev_queue.appendleft(turtlebot_pos)
    return turtlebot_pos

class VisionMetrics:
    def __init__(self):
        self.img = None
        self.turtlebot_proj = None
        self.goal_proj = None
        self.unrefined_corners = None
        self.refined_corners = None
        self.contoured_img = None

    def check_basic_metrics(self):
        if self.img is not None and self.turtlebot_proj is not None and self.goal_proj is not None:
            return True
        return False

    def check_corner_metrics(self):
        if self.refined_corners is not None:
            return True
        return False

class MazeProcess:
    static_transform = np.array([0.014, 0, -0.397])
    def __init__(self, maze_pub_topic, img_sub_topic, cam_info_topic, cam_frame, base_frame):
        self.num_steps = 0
        self.messages = deque([], 5)
        # temp var because ros is cancer
        self.goal_pos = None
        self.corner_tracker = NoiseTracker(tolerance=10, queue_size=20)
        self.turtlebot_tracker = NoiseTracker(tolerance=30, queue_size=7)
        #ar_sub = message_filters.Subscriber('/ar_pose_marker', AlvarMarkers)
        image_sub = message_filters.Subscriber(img_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)
        self.cam_frame = cam_frame
        self.maze_pub = rospy.Publisher(maze_pub_topic, Maze, queue_size=10)
        self.goal_frame = frame_id
        self.base_frame = base_frame
        self.maze = Maze()
        self._bridge = CvBridge()
        self.num_steps = 0

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, caminfo_sub],
                                                          queue_size=100, slop=0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        """ Debugging Stuff """
        self.vision_metrics = VisionMetrics()
            


    def callback(self, image, camera_info):
        try:
            img = self._bridge.imgmsg_to_cv2(image)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cam_matrix = get_camera_matrix(camera_info)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((img, cam_matrix))

    def ar_callback(self, ar_markers):
        found_goal, idx = False, 0
        while not found_goal and idx < len(ar_markers.markers):
            if ar_markers.markers[idx].id == self.goal_frame:
                pos_1 = ar_markers.markers[idx].pose.pose.position
                self.goal_pos = np.array([pos_1.x, pos_1.y, pos_1.z])
                found_goal = True
            else:
                idx+=1
            if not found_goal:
                self.goal_pos = None
        

    def publish_once_from_queue(self):
        #print(self.goal_pos)

        # TODO: Remove dependency on self.goal_pos
        if self.messages:
            self.num_steps += 1
            self.maze.success = True
            img, cam_matrix = self.messages.pop()

            #Test Contrast
            """
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            lab_planes = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            lab_planes[0] = clahe.apply(lab_planes[0])
            lab = cv2.merge(lab_planes)
            img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            """

            #img = cv2.resize(img, (480,640),interpolation=cv2.INTER_CUBIC)
            try:
                turtlebot_transform = self.tfBuffer.lookup_transform(self.cam_frame, self.base_frame, rospy.Time()).transform.translation
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                print(e)
                self.publish_maze(self.maze)
                return

            goal_frame = 'ar_marker_' + str(self.goal_frame)
            
            turtlebot_pos = np.array([turtlebot_transform.x, turtlebot_transform.y, turtlebot_transform.z])
            turtlebot_transformed_pos = np.matmul(cam_matrix,turtlebot_pos)
            turtlebot_img_pos = ((1/turtlebot_transformed_pos[2]) * turtlebot_transformed_pos[:2]).astype(np.int)
            if self.num_steps <= 30:
                seg_img, M = generate_overhead_segment(img, background_color, turtlebot_color, self.corner_tracker)
            else:
                seg_img, M = generate_overhead_segment(img, background_color, turtlebot_color, self.corner_tracker, update_corners=False)
                
            if seg_img is None and M is None:
                print('Error: Segmentation Failed')
                self.publish_maze(self.maze)
                return

            
            turtlebot_proj = warp_pixel(turtlebot_img_pos, M)


            self.maze.turtlebot_pos = update_turtlebot_pos(turtlebot_proj, self.turtlebot_tracker)
            #self.maze.turtlebot_pos = turtlebot_proj
            self.vision_metrics.img = seg_img
            self.vision_metrics.turtlebot_proj = turtlebot_proj
            thresh_img = color_threshold(img, 'green')
            

            try:
                # trans = self.tfBuffer.lookup_transform('base_link', goal_frame, rospy.Time()).transform
                trans = self.tfBuffer.lookup_transform(goal_frame, self.base_frame, rospy.Time()).transform
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                print('Error: Can\'t locate goal {}'.format(goal_frame))
                #msg = Maze()
                self.maze.success = False
                #msg.turtlebot_pos = turtlebot_proj
                self.publish_maze(self.maze)
                return
            

            if self.goal_pos is None:
                print('Error: Can\'t locate goal {}'.format(goal_frame))
                self.maze.success = False
                self.publish_maze(self.maze)
                return

            goal_transformed_pos = np.matmul(cam_matrix,self.goal_pos)
            goal_img_pos = ((1/goal_transformed_pos[2]) * goal_transformed_pos[:2]).astype(np.int)

            
            
            goal_proj = warp_pixel(goal_img_pos, M)

            occupancy_map = build_map(seg_img, *map_size)
            # turtlebot_map_pos, goal_map_pos = localize_goals(turtlebot_proj, goal_proj, map_size, seg_img.shape[:2])
            flattened_map = occupancy_map.flatten()

            self.maze.map_shape = map_size
            # print(type(self.maze.occupancy_grid))
            self.maze.occupancy_grid = list(flattened_map)
            self.maze.transform = trans
            self.maze.goal_pos = goal_proj
            self.maze.seg_img_shape = seg_img.shape[:2]
            self.maze.success = True
            self.publish_maze(self.maze)
            self.vision_metrics.goal_proj = goal_proj
            return thresh_img

    def publish_maze(self, maze):
        try:
            self.maze_pub.publish(maze)
        except rospy.exceptions.ROSSerializationException as e:
            print(e)
            return
            
def main():
    rospy.init_node('maze', anonymous=True)
    maze_process = MazeProcess('maze/grid', '/usb_cam/image_raw', '/usb_cam/camera_info', 'usb_cam', 'base_link')
    #maze_process = MazeProcess('maze/grid', '/camera/color/image_raw', '/camera/color/camera_info', 'camera_rgb_frame', 'maze_link')
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, maze_process.ar_callback)
        debug = maze_process.publish_once_from_queue()
        if maze_process.vision_metrics.check_basic_metrics():
            #im2, contours, hierarchy = cv2.findContours((255-img), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            img = cv2.cvtColor(maze_process.vision_metrics.img, cv2.COLOR_GRAY2BGR)
            turtlebot = maze_process.vision_metrics.turtlebot_proj
            goal = maze_process.vision_metrics.goal_proj
            #print(maze.maze.occupancy_grid)
            grid = np.asarray(maze_process.maze.occupancy_grid).reshape((map_size[0], map_size[1]))
            grid[grid==0] = 255
            grid[grid==1] = 0
            grid[grid==2] = 128
            if turtlebot is not None:
                img[turtlebot[1]:turtlebot[1]+10, turtlebot[0]:turtlebot[0]+10] = [0,0,255]
            if goal is not None:
                img[goal[1]:goal[1]+10, goal[0]:goal[0]+10] = [255,0,0]
            #img = cv2.drawContours(img, contours, -1, (0,255,0), 3)
        
            cv2.imshow('maze image',img)
            cv2.imshow('downsample test', grid)
            if debug is not None:
                cv2.imshow('debug', debug)
            #cv2.imshow('downsample test', np.array(grid, dtype=np.uint8))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        r.sleep()

if __name__ == '__main__':
    main()




