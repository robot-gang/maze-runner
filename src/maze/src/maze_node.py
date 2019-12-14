#!/usr/bin/env python
import argparse
import cv2
import imutils
import message_filters
import rospy
import tf2_ros


from ar_track_alvar_msgs.msg import AlvarMarkers
from collections import deque
from cv_bridge import CvBridge
from detection_utils import *
from maze.msg import Maze
from segmentation_utils import *
from sensor_msgs.msg import Image, CameraInfo


"""
RUNNING INSTRUCTIONS:

rosrun maze maze_node.py 3 4 green red ar_marker_0 ar_marker_1

(rosrun maze maze_node.py grid_shape[0] grid_shape[1] background_color turtlebot_color turtlebot_frame goal_frame)

create minimal viable product (publish empty maze) and then try to speed up computation

"""

class Grid:
    def __init__(self, map_r, map_c):
        self.map_r = map_r
        self.map_c = map_c
        self.grid = np.full((map_r, map_c), 2).astype(np.uint8)
        self.rot = 0

    def update_grid(self, img, turtlebot_img_pos):
        # determine criteria for switching
        # want image reshape to map to shape of image
        
        rows, cols = max(self.map_r, self.map_c), min(self.map_r, self.map_c)
        if img.shape[0] < img.shape[1]:
            rows, cols = cols, rows

        
        occupancy_map = cv2.resize(img, (cols, rows),interpolation=cv2.INTER_CUBIC)
        """
        if np.count_nonzero(self.grid) != 0:
            occupancy_dict = {angle:imutils.rotate_bound(occupancy_map,angle) for angle in [0,90,180,270]}
            similarity = {angle:np.count_nonzero(np.bitwise_and(occupancy_dict[angle], self.grid))
                            if (occupancy_dict[angle].shape == self.grid.shape) else -1 for angle in occupancy_dict}
            

            # get angle corresponding to max_rotation
            max_angle = max(similarity, key=lambda key: similarity[key])
            # I think we want to rotate everything back
            occupancy_map, self.rot = occupancy_dict[max_angle], -max_angle
        """

        occupancy_map[occupancy_map < 118] = 1
        occupancy_map[occupancy_map > 138] = 0
        occupancy_map[occupancy_map > 1] = 2
        self.grid[occupancy_map != 2] = occupancy_map[occupancy_map != 2]
        

    

class NoiseTracker:
    def __init__(self, tolerance=30, queue_size=7):
        self.prev_queue = deque([], queue_size)
        self.tolerance = tolerance
        self.allow_update = True  

class MazeProcess:
    def __init__(self, maze_pub_topic, img_sub_topic, cam_info_topic, cam_frame, base_frame, 
                    wall_color, turtlebot_color, goal_id, marker_id, maze_rows, maze_cols, ar_size):
        # message handling
        self.messages = deque([], 5)
        self.maze_pub = rospy.Publisher(maze_pub_topic, Maze, queue_size=10)
        self.maze = Maze()
        self._bridge = CvBridge()

        # intialize tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # get frame information
        self.goal_id = goal_id
        self.goal_frame = 'ar_marker_' + str(goal_id)

        self.marker_id = marker_id
        self.marker_frame = 'ar_marker_' + str(marker_id)
        self.turtlebot_color = turtlebot_color

        self.cam_frame = cam_frame
        self.base_frame = base_frame
        self.wall_color = wall_color
        # counter for number of callbacks
        self.num_steps = 0

        #AR Marker attributes
        self.visible_markers = AlvarMarkers()
        self.ar_size = ar_size

        #Median Filter objects (noise reduction)
        self.corner_tracker = NoiseTracker(tolerance=10, queue_size=20)
        self.turtlebot_tracker = NoiseTracker(tolerance=30, queue_size=7)

        #Grid object for handling updates
        self.grid = Grid(maze_rows, maze_cols)

        # initialize queue for image and camera sync
        image_sub = message_filters.Subscriber(img_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, caminfo_sub],
                                                          queue_size=100, slop=0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

        
    # callback for image and camera information processing
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
        self.visible_markers = ar_markers

    # function that gets transform between two frames (do it a lot)
    def get_frame_transform(self, target_frame, source_frame, tag_id, id_list, set_flag=False):
        if tag_id in id_list:
            try:
                trans = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time()).transform
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                print(e)
                if set_flag:
                    self.maze.success = False
                self.maze_pub.publish(self.maze)
                return
            return trans
        else:
            self.maze.success = False
            self.maze_pub.publish(self.maze)
        

    # main logic for publishing handled here
    def publish_from_queue(self):
        if self.messages:
            img, cam_matrix = self.messages.pop()
            id_list = [marker.id for marker in self.visible_markers.markers]

            # get all  necessary transforms
            turtlebot_transform = self.get_frame_transform(self.cam_frame, self.base_frame, self.marker_id, id_list)
            if turtlebot_transform is None: return
            
            turtlebot_pos = translation_from_transform(turtlebot_transform)
            turtlebot_img_pos = get_image_coords(turtlebot_pos,cam_matrix)

            marker_transform = self.tfBuffer.lookup_transform(self.cam_frame, self.marker_frame, rospy.Time()).transform
            marker_pos = translation_from_transform(marker_transform)
            marker_img_pos = get_image_coords(marker_pos, cam_matrix)
            marker_corners = get_artag_corners(marker_transform, self.ar_size)
            img_marker_corners = [get_image_coords(c, cam_matrix) for c in marker_corners]
            
            if self.num_steps <= 30:
                seg_img, M = generate_overhead_segment(img, img_marker_corners, self.wall_color, self.turtlebot_color, self.corner_tracker)
            else:
                seg_img, M = generate_overhead_segment(img, img_marker_corners, self.wall_color, self.turtlebot_color, self.corner_tracker, update_corners=False)
            
            turtlebot_proj = warp_pixel(turtlebot_img_pos, M)
            marker_proj = warp_pixel(marker_img_pos, M)
            self.maze.turtlebot_pos = update_turtlebot_pos(turtlebot_proj, self.turtlebot_tracker)
            self.maze.marker_pos = marker_proj
            
            
            maze_transform = self.get_frame_transform(self.goal_frame, self.base_frame, self.goal_id, id_list, set_flag=True)
            if maze_transform is None: return

            goal_transform = self.get_frame_transform(self.cam_frame, self.goal_frame, self.goal_id, id_list)
            # get 3d spacial postions for each object of interest
                        
            goal_pos = translation_from_transform(goal_transform)
            goal_img_pos = get_image_coords(goal_pos, cam_matrix)
            goal_proj = warp_pixel(goal_img_pos, M)

            
            marker_pos = translation_from_transform(marker_transform)
            
            self.grid.update_grid(seg_img, turtlebot_img_pos)
            
            if self.grid.rot != 0:
                # rotate goal position and turtlebot position back
                center = (int(seg_img.shape[0]/2)+1, int(seg_img.shape[1]/2)+1)
                angle = self.grid.rot
                R_matrix = cv2.getRotationMatrix2D(center, float(angle), 1.0)
                self.maze.turtlebot_pos = np.matmul(M, np.hstack((turtlebot_proj,1)))[:-1].astype(int)
            
            
            self.maze.map_shape =[self.grid.map_r, self.grid.map_c]
            occupancy_grid = self.grid.grid.copy()
            occupancy_grid[occupancy_grid == 2] =1
            self.maze.occupancy_grid = list(occupancy_grid.flatten())
            self.maze.transform = maze_transform
            self.maze.goal_pos = goal_proj
            self.maze.seg_img_shape = seg_img.shape[:2]
            self.maze.success = True
            self.maze_pub.publish(self.maze)
            return seg_img


   
def update_turtlebot_pos(turtlebot_pos, tracker):
    if tracker is not None and len(tracker.prev_queue) > 0:
        median = np.median(np.asarray(tracker.prev_queue), axis=0)
        # print(median)
        tracker.prev_queue.appendleft(turtlebot_pos)
        if abs(turtlebot_pos[0]-median[0]) > tracker.tolerance and abs(turtlebot_pos[1]-median[1]) > tracker.tolerance:
            return median
    tracker.prev_queue.appendleft(turtlebot_pos)
    return turtlebot_pos

def generate_overhead_segment(img, ar_corners, background_color, turtlebot_color, corner_tracker=None, update_corners=True):
    img = cv2.GaussianBlur(img, (7,7), 1)
    threshold_img = color_threshold(img, background_color)
    kernel = np.ones((10,10),np.uint8)
    #threshold_img = cv2.morphologyEx(threshold_img, cv2.MORPH_CLOSE, kernel)
    points = locate_potential_corners(threshold_img, k=0.04, res_sel=2)

    """ find corners """
    if update_corners:
        corners = find_four_corners(points, corner_tracker)
    else:
        corner_test = np.asarray(corner_tracker.prev_queue)
        corners = np.median(corner_test, axis=0)

    
    """ project image into overhead perspective """
    # this projection is on the original image to find the turtlebot
    M, max_w, max_h = four_point_transform(img, corners)

    transformed_img = warp_image(img, M, max_w, max_h)
    proj_img = warp_image(threshold_img, M, max_w, max_h)

    # this projection is on the thresholded image to form the maze of the segmented image
    for idx in range(4):
        ar_corners[idx] = warp_pixel(ar_corners[idx], M)
    """ localize turtle bot """
    turtlebot_img = localize_turtlebot(transformed_img, ar_corners, turtlebot_color, kernel)

    """ color regions in projected image covered by turtlebot"""
    pts = np.asarray(np.where(turtlebot_img==0)).T
    proj_img[np.where(turtlebot_img == 0)] = 128

    return proj_img, M     



def main():
    # Argument Parsing
    parser = argparse.ArgumentParser()
    parser.add_argument('maze_rows', help='Number of rows in maze.', type=int)
    parser.add_argument('maze_cols', help='Number of columns in maze', type=int)
    parser.add_argument('wall_color', help='String specifiying the color of the maze')
    parser.add_argument('turtlebot_color', help='String specifying the color of the turtlebot')
    parser.add_argument('goal_ar_num', help='Number corresponding to AR tag on goal', type=int)
    parser.add_argument('turtlebot_ar_num', help='Number corresponding to AR tag on turtlebot', type=int)
    parser.add_argument('artag_size', help='Float corresponding to size of AR tag (or AR tag paper) (cm)', type=float)

    args = parser.parse_args()

    rospy.init_node('maze', anonymous=True)
    maze_process = MazeProcess('maze/grid', '/usb_cam/image_raw', '/usb_cam/camera_info', 'usb_cam', 'base_link', 
                                                args.wall_color, args.turtlebot_color, args.goal_ar_num, 
                                                args.turtlebot_ar_num, args.maze_rows, args.maze_cols, args.artag_size)
    
    # main publishing loop
    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, maze_process.ar_callback)
        img = maze_process.publish_from_queue()
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            turtlebot = maze_process.maze.turtlebot_pos
            goal = maze_process.maze.goal_pos
            marker = maze_process.maze.marker_pos
            if turtlebot is not None:
                try:
                    img[turtlebot[1]:turtlebot[1]+10, turtlebot[0]:turtlebot[0]+10] = [0,0,255]
                except TypeError as e:
                    print(e)
                    print(turtlebot)
            if goal is not None:
                img[goal[1]:goal[1]+10, goal[0]:goal[0]+10] = [255,0,0]
            
            if marker is not None:
                img[marker[1]:marker[1]+10, marker[0]:marker[0]+10] = [150,0,150]


            if maze_process.grid.grid is not None:
                grid = maze_process.grid.grid.copy()
                grid[grid==0] = 255
                grid[grid==1] = 0
                grid[grid==2] = 128

            cv2.imshow('maze image',img)
            cv2.imshow('occupancy_map', grid)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        r.sleep()





if __name__ == '__main__':
    main()
