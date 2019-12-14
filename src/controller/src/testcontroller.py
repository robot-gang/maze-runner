import numpy as np

def controller(path, orientation):
        while len(path) > 1:
            start = path.pop(0)
            end = path[0]
            rotate, orientation = cmd_rotate(start, end, orientation)
            print(rotate)
            #pub.publish(rotate)

            #r.sleep()
            translate = cmd_translate(start, end)
            print(translate)
            #pub.publish(translate)
            #r.sleep()

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
    cmd.az = angle

    return cmd, curr_orientation

def cmd_translate(start, end):
    cmd = Twist()
    cmd.lx = end[0] - start[0]
    cmd.ly = end[1] - start[1]

    return cmd

class Twist:
    def __init__(self, lx = 0, ly = 0, lz = 0, ax = 0, ay = 0, az = 0):
        self.lx = lx
        self.ly = ly
        self.lz = lz
        self.ax = ax
        self.ay = ay
        self.az = az

    def __str__(self):
        return str((self.lx, self.ly, self.lz, self.ax, self.ay, self.az))



path = [(0, 0), (1, 0), (2, 1), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4), (3, 5), (2, 5), (1, 5), (1, 4), (0, 4)]
orientation = 0
controller(path, orientation)