---
layout: page
title: Design
permalink: /design
---

---
# Table of Contents
1. [Overview](#overview)
2. [Vision](#vision)
    - [Our Design and Decisions](#our-design-and-decisions)
    - [Design Choices and Trade-Offs](#design-choices-and-trade-offs)
    - [Analysis of Design](#analysis-of-design)
3. [Planning and Control](#planning-and-control)
    - [Our Design and Decisions](#our-design-and-decisions-1)
    - [Design Choices and Trade-Offs](#design-choices-and-trade-offs-1)
    - [Analysis of Design](#analysis-of-design-1)

---

## Overview

Because our goal from the beginning was to eventually make the path planning and control dynamic, we needed to constantly check the maze to see if the goal or the shortest path changed. At the general level, we decided to make a vision node, a planning node, and a control node. 
- The **vision** node would continuously receive the video feed from the camera, process it into a maze, and publish the maze. This node would also calculate appropriate transforms and pixel positions for control and publish those too. 
- The **planning** node would receive the maze and return a path through the maze.
- The **control** node would use the information from both the planning and vision nodes to set waypoints for the robot to follow, and would publish velocities to help the robot reach the waypoints. This would allow the robot to solve the maze

<center><img src="assets/design/system-diagram.jpg" width="75%"></center>
*<center><sub><sup>Diagram showing general overview of system</sup></sub></center>*



[(back to top)](#table-of-contents)

## Vision

When we initially went about designing this project, we wanted to be super lazy about setting up our environment. Ideally we would have liked to have an overhead camera, but that was really difficult to do logistically, and was also kind of limiting. We also did not want spend a lot of time moving our camera to a "perfect" position, or spend a lot of time setting up the maze. As a result, the vision needed to be quite robust to changes in our setup. Because we continuously ran the vision, it needed to be fast (work in realtime). While there were a few contraints we had to enforce on our setup, our group's design gave us a lot of freedom in setting up our maze.

[(back to top)](#table-of-contents)

# Our Design and Decisions

The system for the computer vision is quite complicated, so a general outline of the system is provided to make it more readable.
1. Segment image using color thresholding to get the walls of the maze.
    - Thresholds for colors picked through trial and error.
    - Works best if the markers contrast strongly with everything else in the camera view.
    - Apply morphological transform in Opencv to remove noise in image.
2. Assume maze is going to be bounded by bounded by a well-formed rectangle.
    - Not strictly necessary in that only the pieces of paper corresponding to the rectangle's corners need to exist.
    - If there are boundaries, however, they need to be relatively straight and not too spaced apart ideally.
3. Detect potential clusters of corners in segmented image, and refine them to get corner pixels.
4. Use K-means clustering on convex hull of corners to separate points into four groupings.
    - Depending on the angle of the camera, the shape of the maze can look like a trapezoid, parallelogram, rectangle, etc.
    - Depending on distance of camera to the maze, maze can also be translated.
    - Difficult to find a good method to classify outermost corners of maze after classification.
    - Becomes easier to find a method if we only consider clusters in a local region.
    - K-means allows a flexible way to split the maze into four quadrants, regardless of the camera's position or orientation.

    <center><img src="assets/design/kmeans.png" width="75%"></center>
    *<center><sub><sup>Illustration of how effective K-means is at splitting maze.</sup></sub></center>*

5. In each cluster, pick point that is furthest away from center of convex hull of points.
    - somewhat problematic but works most of the time.
6. Have (ideally) found four corners of a rectangle, apply median filter to corners to reduce chance of outliers.
7. Find maximum bounding rectangle
8. Retrieve perspective transform matrix, and apply to segmented image and original image.
9. Segment turtlebot in the transformed original image.
    - Use color thresholding on turtlebot (black for base and whatever color the Turtlebot is)
    - Segment AR Tag on turtlebot by calculating corners of tag in AR tag frame, transforming to camera frame, transforming to pixel space, transforming to warped pixel space, and then drawing a rotated rectangle contour around corners using OpenCV.
10. Overlay segmented Turtlebot with segmented maze.
11. Downsample image to get 2D-grid for path planning.
12. Assign walls a 1, open space a 0, and the region covered by the Turtlebot as a 2 (internally).
13. Compare to internalized grid (initially all 2s).
14. Only only change internal grid points' values if new corresponding grid point is a 1 or 0
    - If turtlebot drives and blocks a portion of the maze we knew prior, safest to assume it does not change as the Turtlebot blocks it from the camera's view.
    - If Turtlebot moves and no longer blocks part of the maze, we now know what that that part of the maze looks like, and there is not reason to make it uncertain.
    - Only change maze if someone alters the walls.
15. Publish uncertain regions as 1 (wall). 
    - Requires large assumption
16. Get pixel position of goal, base of Turtlebot, and AR tag on turtlebot using TF and applying camera intrinsic matrix.
17. Get transform between Turtlebot and Goal using TF.
18. Publish updated internal grid, pixel positions, and Transformations


Explain general process for vision
Notable things to talk about
K-Means to find corners
Segment turtlebot/AR tag 
Median Filter corners


[(back to top)](#table-of-contents)

# Design choices and trade-offs

Motivate decision to use grid-based planning
Motivate decision to use perspective
Trade off in updating corners
Explain trade off with speed vs robustness


[(back to top)](#table-of-contents)

# Analysis of Design

Discuss strengths and limitations of current design
-> need all four corners
-> no easy way to track rotations
-> gridding is very naive, should use probabilistic model


[(back to top)](#table-of-contents)

## Planning and Control

The planner should be able to find the shortest path from the starting position to the goal position. To be robust, a planner must correctly handle cases like no path and multiple paths. The controller brings the TurtleBot to the desired position while following the shortest path returned by a planner.  We planned to create two controllers. One is a fixed path controller, which assumes that the environment of the TurtleBot stays invariant. The planner will compute the shortest path the first time when it receives a valid TurtleBot maze configuration, and the system will memorize the path. Another one is a dynamic path controller, which assumes that the environment and the goal position are changeable. To cope with the change, the controller requires the planner to update the path whenever it receives a TurtleBot maze configuration.

[(back to top)](#table-of-contents)

# Our Design and Decisions

We used A* algorithm to compute the shortest path. There were two reasons to choose A*. First, A* is deterministic and optimal if the heuristic is consistent and admissible. Second, the computation is very fast when the TurtleBot maze configuration is small.  The heuristic was euclidean distance because diagonal motion was allowed.
The core of control was a proportional controller. At each turning point of the path, the desired direction changed. Hence, turning to the desired angle prohibited the error accumulation.

[(back to top)](#table-of-contents)

# Design choices and trade-offs

After computing the shortest path, we first removed all the non-turning points. The motivation to develop a fixed path controller was to split the path into a sequence of shorter routes. Each route had two turning points, and the motion along that route was a pure rotation and pure translation. The pure rotation at the turning point had two main functionalities. One was to align the TurtleBot to the desired orientation, and the other was for error correction. We chose the goal frame as the fixed frame so that the look_up_transofrm function from tf package could return the orientation to the goal frame. For each route, we computed the desired angle in the fixed frame, then we used proportional control to correct the orientation of the TurtleBot. In dynamic path control, the main difference was that we updated the maze TurtleBot configuration and the corresponding path frequently. The challenge to adapt the fixed path controller to the dynamic path controller was that TurtleBot started turning too early. Due to the way how we calculated the grid location, the TurtleBot started turning once the center of it entered a grid, not necessarily at the center of the grid. To solve it, we defined a threshold near the center of a grid (turning point) so that the state (x, y, orientation) only got updated when the center of the TurtleBot was close to the center of the grid. The problem induced was that if the TurtleBot was not close enough to the center of a turning point, it still kept the state of the previous turning point. To further solve the dynamic control, we needed to implement a more sophisticated state update rule.

[(back to top)](#table-of-contents)


# Analysis of Design

The fixed path controller worked very well because our assumption oversimplified the problem. One can treat the fixed path controller the same as how we figure out the path from home to school without a GPS. Without any information update, we choose the shortest/fastest path from a memorized map. It performs as good as how we can find the lab every day. There were two reasons we might want to work on dynamic control. First, the goal position might change. As an example, one might change his mind on the way to Moffitt library and decide to study at the Engineering library. Second, the path or the environment might change because some road construction might block your way to the Engineering library. Therefore, the TurtleBot also needed to adjust its path frequently. Due to the simplified state update rule, the dynamic controller was not robust. When we changed the goal or maze, the TurtleBot worked best at a turning point, it worked okay near the center of a grid, and it barely worked near the edges of grids because it was confused about its location.

[(back to top)](#table-of-contents)
