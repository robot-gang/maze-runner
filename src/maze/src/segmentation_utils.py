import cv2
import numpy as np

from skimage.measure import block_reduce
from sklearn.mixture import GaussianMixture
from scipy.misc import imresize
import rospy
import tf2_ros
import tf
import tf_conversions.posemath as pm
from scipy.spatial.transform import Rotation as R 

def get_artag_corners(transform, artag_size):
    artag_size /= 200 # convert to meters
    
    translation = np.array([[transform.translation.x, transform.translation.y, transform.translation.z]])
    rotation = R.from_quat(np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]))
    rot_matrix = rotation.as_dcm()

    # get corners of artag in home coordinates
    tr = [artag_size, artag_size, 0]
    tl = [-artag_size, artag_size, 0]
    bl = [-artag_size, -artag_size, 0]
    br = [artag_size, -artag_size, 0]
    corners = [tr, tl, br, bl]        
    #print(rot_matrix.shape, translation.T.shape)
    trans_matrix = np.vstack((np.hstack((rot_matrix, translation.T)), np.array([0,0,0,1])))
    transformed_corners = [np.matmul(trans_matrix, np.block([np.asarray(corners[idx]), 1]))[:-1] for idx in range(4)]
    return transformed_corners

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

def localize_turtlebot(img, ar_corners, color, kernel):
    rect = cv2.minAreaRect(np.asarray(ar_corners))
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    turtlebot_img = color_threshold(img, color)
    base_img = color_threshold(img, 'black')
    thresh_img = turtlebot_img
    thresh_img = cv2.bitwise_and(turtlebot_img,base_img)
    thresh_img = cv2.drawContours(thresh_img, [box],-1,[0], -1)

    #dilated_img = cv2.morphologyEx(thresh_img, cv2.MORPH_CLOSE, kernel)
    dilated_img = thresh_img

    return dilated_img
