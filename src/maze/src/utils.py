""" Stores Utility Functions """
import cv2
import sys
import itertools
import numpy as np

import scipy.spatial.qhull as qhull
from scipy.misc import imresize
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from skimage import filters
from sklearn.mixture import GaussianMixture
from sklearn.cluster import KMeans
from skimage.measure import block_reduce
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

def display_img(img):
    """displays image to screen and exits on keypress

    Parameters
    ---------
    img : ndarray
        image array
    
    Returns
    -------
        None, displays image and exits on keypress
    """
    cv2.imshow('test', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# simple grayscale threshold segmentation
def threshold_segment(img):
    """perform basic grayscale thresholding Otsu's binarization method

    Parameters
    ---------
    img : ndarray
        grayscale image array
    
    Returns
    -------
    ndarray
        segmented black and white img
    """
    #seg_img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    if len(img.shape) > 2:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret2,seg_img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return seg_img

# image segmentation using the EM MoG Algorithm (very slow)
def cluster_segment(img, n_clusters, random_state=0):
    """perform segmentation using the EM MoG clustering algortihm

    Parameters
    ---------
    img : ndarray
        colored image array

    n_clusters : int
        integer specifying number of classes

    random_state : int
        integer specifying seed for starting parameters
    
    Returns
    -------
    ndarray
        segmented grayscale img (by cluster)
    """
    img_d = block_reduce(img, block_size=(2, 2, 1), func=np.mean)
    img_r = np.reshape(img_d,(img_d.shape[0]*img_d.shape[1], img_d.shape[2]))
    em = GaussianMixture(n_components=n_clusters, random_state=random_state).fit(img_r)
    clusters = em.predict(img_r)
    cluster_img = np.reshape(clusters, img_d.shape[:-1])
    img_u = imresize(cluster_img, (img.shape[0], img.shape[1]), interp='nearest')
    return img_u.astype(np.uint8)

# do color thresholding
def color_threshold(img, color='white'):
    """perform thresholding based on color

    Parameters
    ---------
    img : ndarray
        colored image array

    color : string
        string indicating color
    
    Returns
    -------
    ndarray
        segmented black and white img (by color)
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([0,0,168])
    upper = np.array([172,111,255])
   
    #Add if condition for different colors
    #not good enough
    if color.lower() == 'red':
        lower1 = np.array([0,50,20])
        upper1 = np.array([5,255,255])
        lower2 = np.array([175,50,20])
        upper2 = np.array([180,255,255])
        mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)
    else:
        if color.lower() == 'green':
            #lower = np.array([50,10,70])
            #upper = np.array([80,255,255])
            lower = np.array([36,20,70])
            upper = np.array([85,255,255])
        elif color.lower() == 'black':
            lower = np.array([0,0,0])
            upper = np.array([180, 255, 45])
        elif color.lower() == 'yellow':
            lower = np.array([20, 100, 100])
            upper = np.array([40, 255, 255])
        else:
            print('Error: Invalid color argument. {} currently not supported'.format(color.lower()))
            print('Defaulting to black for thresholding.')
            lower = np.array([0,0,0])
            upper = np.array([180, 255, 45])
        mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_not(cv2.bitwise_and(hsv, hsv, mask = mask))
    gray_result = cv2.split(result)[2]
    gray_result[gray_result < 255] = 0
    return gray_result


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

    
    
def localize_turtlebot(img, color, kernel):
    turtlebot_img = color_threshold(img, color)
    base_img = color_threshold(img, 'black')
    thresh_img = turtlebot_img
    thresh_img = cv2.bitwise_and(turtlebot_img,base_img)
    #dilated_img = cv2.morphologyEx(thresh_img, cv2.MORPH_CLOSE, kernel)
    dilated_img = thresh_img

    return dilated_img

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

    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]], dtype = "float32")
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(sorted_pts, dst)
    warped = cv2.warpPerspective(img, M, (maxWidth, maxHeight))

    # return the warped image
    return warped, M

def warp_pixel(pixel, M):
    homog_pix = np.hstack((pixel,1))
    norm_term = np.matmul(M[2], homog_pix)
    x_term = np.matmul(M[0], homog_pix)
    y_term = np.matmul(M[1], homog_pix)
    dst = np.array([x_term/norm_term, y_term/norm_term]).astype(np.int)
    return dst


def generate_overhead_segment(img, background_color, turtlebot_color, corner_tracker=None, update_corners=True):
    img = cv2.GaussianBlur(img, (7,7), 1)
    #kernel = np.ones((5,5),np.float32)/25
    #img = cv2.filter2D(img,-1,kernel)
    threshold_img = color_threshold(img, background_color)
    kernel = np.ones((10,10),np.uint8)
    threshold_img = cv2.morphologyEx(threshold_img, cv2.MORPH_CLOSE, kernel)

    """ locate potential corner regions """
    points = locate_potential_corners(threshold_img, k=0.04, res_sel=2)

    """ find corners """
    if update_corners:
        corners = find_four_corners(points, corner_tracker)
    else:
        corner_test = np.asarray(corner_tracker.prev_queue)
        corners = np.median(corner_test, axis=0)
    #print(corners)
    #print(points[:,0], points[:,1])
    if corners is None:
        return None, None
    
    """ project image into overhead perspective """
    # this projection is on the original image to find the turtlebot
    tranformed_img, _ = four_point_transform(img, corners)

    # this projection is on the thresholded image to form the maze of the segmented image
    proj_img, warp_tranform = four_point_transform(threshold_img, corners)
    
    """ localize turtle bot """
    #kernel1 = np.ones((7,7),np.uint8)
    turtlebot_img = localize_turtlebot(tranformed_img, turtlebot_color, kernel)

    """ color regions in projected image covered by turtlebot"""
    pts = np.asarray(np.where(turtlebot_img==0)).T
    centroid = np.mean(pts, 0).astype(int)
    proj_img[np.where(turtlebot_img == 0)] = 128

    return proj_img, warp_tranform

"""
def build_map(img, map_x, map_y):
    

    if img.shape[0] > img.shape[1]:
        x = max(map_x, map_y)
        y = min(map_x, map_y)
    else:
        x = min(map_x, map_y)
        y = max(map_x, map_y)
        
    occupancy_map = np.zeros((x,y))

    for i in range(x):
        for j in range(y):
            lower_x = int(img.shape[0]/x)*i
            upper_x = min(int(img.shape[0]/x)*(i+1), img.shape[0])
            lower_y = int(img.shape[1]/y)*i
            upper_y = min(int(img.shape[1]/y)*(i+1), img.shape[1])

            grid = img[lower_x:upper_x,lower_y:upper_y]
            uncertainty = len(grid[grid==128])
            free = len(grid[grid==255])
            occupied = len(grid[grid==0])
            occupancy_map[i,j] = np.round((occupied+0.5*uncertainty)/(grid.shape[0]*grid.shape[1]))

    return occupancy_map.astype(np.int8)
"""
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

def draw_contours(img):
    # Grayscale 
    test_img = img.copy()
    gray = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY) 
  
    # Find Canny edges 
    edged = cv2.Canny(gray, 30, 200) 
    
    # Finding Contours 
    # Use a copy of the image e.g. edged.copy() 
    # since findContours alters the image 
    contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    
    # Draw all contours 
    # -1 signifies drawing all contours 
    cv2.drawContours(test_img, contours, -1, (0, 255, 0), 3) 
    return test_img


""" Driver """

if __name__ == '__main__':
    background_color = sys.argv[3]
    turtlebot_color = sys.argv[4]

    # goes turtlebot frame, then goal frame
    frame_ids = map(int, sys.argv[5:7])
    map_size = map(int, sys.argv[1:3])
    cap = cv2.VideoCapture(-1)
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        print(frame.shape)
        #seg_img, turtlebot_proj, goal_proj = generate_overhead_segment(frame, background_color, turtlebot_color)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lab_planes = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        lab_planes[0] = clahe.apply(lab_planes[0])
        lab = cv2.merge(lab_planes)
        bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        cv2.imshow('original image', bgr)
        #cv2.imshow('selected corners',test_img1)
        #cv2.imshow('projected image', tranformed_img)
        #cv2.imshow('refined corners', test_img2)
        #cv2.imshow('refined corners', test_img1)
        #cv2.imshow('turtlebot image', seg_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

