""" Stores Utility Functions """
import cv2
import sys
import itertools
import numpy as np

import scipy.spatial.qhull as qhull
from scipy.misc import imresize
from scipy.spatial import ConvexHull, convex_hull_plot_2d, distance
from skimage import filters
from sklearn.mixture import GaussianMixture
from sklearn.cluster import KMeans
from skimage.measure import block_reduce
#from segmentation_utils import *
import matplotlib.pyplot as plt



def get_camera_matrix(camera_info_msg):
    """given a ros CameraInfo message, returns the K matrix required to 
    transform a point in 3D to an image pixel.

    Parameters
    ---------
    camera_info_msg : CameraInfo
        CameraInfo message from subscribed topic
    
    Returns
    -------
        ndarray
            3x3 intrinsic matrix containing calibrated camera information
            Specific definition can be found here:
            http://ksimek.github.io/2013/08/13/intrinsic/

    """
    K = camera_info_msg.K
    return np.reshape(K, (3,3))




def locate_potential_corners(img, block_size=11, k_size=11, k=0.04, threshold_sel=0.01, res_sel = 1):
    """locates potential corner pixels given a segmented image

    Parameters
    ---------
    img : ndarray
        grayscale image array

    block_size : int
        side length of square region (neighborhood) considered for corner detection
    
    k_size : int
        determines size of the Sobel kernel
        - larger k_size will make the edges in initial edge detection more blurry

    k : float
        free parameter in corner detection equation
        - basically trades off between precision and recall
        - bigger k gives fewer false corners, but also fewer true corners
        - smaller k gives more corners in general, but more false corners

    threshold_sel : float
        the opencv corner detection algorithm returns a transformed image
        -this parameter thresholds the transformed image to drive pixels that
        are unlikely to be corners to be zero.
        -parameter is fraction of corner pixel value with largest confidence

    res_sel : int (0 or 1)
        corner refinement returns corner points and the centroid of corner regions
        - 0 gives centroid regions (sometimes more robust to perturbation)
        - 1 gives corner predictions
        generally set to 1
    
    Returns
    -------
    ndarray
        numpy array of coordinates that give the indices of corner points in the image
    """
    test_img = np.float32(img)
    dst = cv2.cornerHarris(test_img,block_size,k_size,k)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,threshold_sel*dst.max(),255,0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    test_corners = cv2.cornerSubPix(test_img,np.float32(centroids),(5,5),(-1,-1),criteria)

    # Now draw them
    res = np.hstack((centroids,test_corners))
    res = np.int0(res)
    if res_sel == 1:
        points = res[:,:2]
    else:
        points = res[:,2:]
    points[:,[1,0]] = points[:,[0,1]]
    return points

def test_harris_corners(img, blockSize=5, ksize=11, k=0.04, threshold_sel=0.01):
    """
    This function is used to get the initial return of the potential corners detected
    by the Harris Corner Detector (not refined).

    (used primarily for debugging corner detector)

    Parameters
    ---------
    (almost identical to parameters in locate_potential_corners so look at those)

    Return
    ---------
    ndarray
        grayscale thresholded image with corner points colored white and non-corner points
        black
    """
    test_img = np.float32(img)
    dst = cv2.cornerHarris(test_img,blockSize,ksize,k)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,threshold_sel*dst.max(),255,0)
    dst = np.uint8(dst)
    return dst


def find_four_corners(potential_corners, corner_tracker = None):
    """given a list of potential corner points, finds the four outermost corners and orders
    them as (top-left corner, bottom-left corner, top-right corner, bottom-right corner)

    Parameters
    ---------
    potential_corners : ndarray
        numpy array of coordinates that give the indices of corner points in the image

    prev_queue : (NOT IMPLEMENTED - IGNORE FOR NOW) ndarray or queue contraining ndarrays
        numpy array of detected corner points in previous iteration of algorithm
        -used to potentially control random large deviations in corners due to noise 

    Return
    ---------
    ndarray
        numpy array containing four ordered corner coordinates
    """
    try:
        hull = ConvexHull(potential_corners)
        cx = np.mean(hull.points[hull.vertices,0])
        cy = np.mean(hull.points[hull.vertices,1])
        potential_corners = potential_corners[hull.vertices]
    except qhull.QhullError as e:
        print(e)
        return None

    #print(potential_corners.shape)
    if potential_corners.shape[0] < 4:
        print('yikers, can\'t find enough corners to do kmeans')
        return None
    centroid = np.array([cx,cy]) #get center of convex hull
    distances = np.sum(np.abs(potential_corners-centroid)**2,axis=-1)**(1./2)

    ## DEBUG: Cancer KMeans implementation, need error handling for clustering
    pairings = zip(potential_corners, distances)
    
    clusters = KMeans(n_clusters = 4, random_state=0).fit_predict(potential_corners)
    classification = zip(clusters, pairings)
    cluster_dict = {key:[val[1] for val in classification if val[0] ==key] for key in range(4)}
    corners = []
    for key in cluster_dict:
        clustered_pts = cluster_dict[key]
        corner = max(clustered_pts, key=lambda elem: elem[1])
        corners.append(corner[0])
    
    # sort by x to get left and right side of image
    corners.sort(key=lambda elem: elem[1])

    #sort upper and lower left
    corners[:2] = sorted(corners[:2], key=lambda elem: elem[0])

    #sort upper and lower right
    corners[2:] = sorted(corners[2:], key=lambda elem: elem[0])

    # Ordering: tl, bl, tr, br
    corners = np.asarray(corners)
    corners[:,[0,1]] = corners[:,[1,0]]
    set_corner = True

    if corner_tracker is not None and corner_tracker.prev_queue is not None and len(corner_tracker.prev_queue) > 0: 
        corner_test = np.asarray(corner_tracker.prev_queue)
        med_corners = np.median(corner_test, axis=0)
        corner_tracker.prev_queue.appendleft(corners)
        return med_corners
    else:
        corner_tracker.prev_queue.appendleft(corners)
        return corners

    
    

def four_point_transform(img, pts):
    """given four corners of a rectangular shape viewed from an 
    angle in image, computes and returns a topdown view

    Parameter
    ---------
    img : ndarray
        image array (can be color, grayscale, or segmented)
    
    pts : list or ndarray
        list of 2D coordinates that form corners of a rectangle (viewed from angle) with the following ordering:
        (top-left corner, bottom-left corner, top-right corner, bottom-right corner)
    Returns
    -------
    ndarray
        reconstructed top-down view of recangular object
    """
    # obtain a consistent order of the points and unpack them
    # individually
    (tl, bl, tr, br) = pts
    sorted_pts = np.array([tl, tr, br, bl], dtype = "float32")

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))


    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    dst = np.array([[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]], dtype = "float32")
    M = cv2.getPerspectiveTransform(sorted_pts, dst)

    # return the warped image
    return M, maxWidth, maxHeight

def warp_image(img, M, max_width, max_height):
    return cv2.warpPerspective(img, M, (max_width, max_height))

def warp_pixel(pixel, M):
    homog_pix = np.hstack((pixel,1))
    norm_term = np.matmul(M[2], homog_pix)
    x_term = np.matmul(M[0], homog_pix)
    y_term = np.matmul(M[1], homog_pix)
    dst = np.array([x_term/norm_term, y_term/norm_term]).astype(np.int)
    return dst


def build_map(img, map_x, map_y):
    if img.shape[0] > img.shape[1]:
        map_x, map_y = map_y, map_x

    occupancy_map = cv2.resize(img, (map_y,map_x),interpolation=cv2.INTER_CUBIC)
    #occupancy_map = cv2.cvtColor(occupancy_map, cv2.COLOR_BGR2GRAY)
    occupancy_map[occupancy_map < 118] = 1
    occupancy_map[occupancy_map > 138] = 0
    occupancy_map[occupancy_map > 1] = 0
    return occupancy_map


def localize_goals(turtlebot_pos, goal_pos, map_size, img_shape):
    """
    turtlebot_pos = (column, row)
    """

    turtlebot_grid_row = int((float(turtlebot_pos[1]) / img_shape[0]) * map_size[0])
    turtlebot_grid_col = int((float(turtlebot_pos[0]) / img_shape[1]) * map_size[1])
    turtlebot_grid = np.array([turtlebot_grid_row, turtlebot_grid_col])

    goal_grid_row = int((float(goal_pos[1]) / img_shape[0]) * map_size[0])
    goal_grid_col = int((float(goal_pos[0]) / img_shape[1]) * map_size[1])
    goal_grid = np.array([goal_grid_row, goal_grid_col])

    return turtlebot_grid, goal_grid


def get_image_coords(point, cam_matrix):
    trans_point = np.matmul(cam_matrix, point)
    img_point = ((1/trans_point[2]) * trans_point[:2]).astype(np.int)
    return img_point


def order_points(points):
    # sort by x to get left and right side of image
    points.sort(key=lambda elem: elem[1])

    #sort upper and lower left
    points[:2] = sorted(points[:2], key=lambda elem: elem[0])

    #sort upper and lower right
    points[2:] = sorted(points[2:], key=lambda elem: elem[0])
    points[1], points[2] = points[2], points[1]
    return points

def translation_from_transform(transform):
    return np.array([transform.translation.x, transform.translation.y, transform.translation.z])