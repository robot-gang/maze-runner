---
layout: page
title: Implementation
permalink: /implementation
---

## Vision
There were two AR tags, one will be on the TurtleBot, the other one will put in the maze. With the help of the camera on the side to provide information of the maze and the position of the TurtleBot and the goal by using both AR tags.

<img src="assets/Implementation/camera_provided_img.png" width="100%">

Using the image provided by the camera, we transformed it into a binary image.

<img src="assets/Implementation/binary_img.png" width="100%">

The binary image made it easier to detect corners of the maze, and then using OpenCV transform the image to a top down view.

<img src="assets/Implementation/actual_corners.png" width="100%">

We also used static transform to provide the actual position of the TurtleBot(the "base_link") since the AR tag is on the TurtleBot.

As shown below, the blue dot is the position of the goal AR tag, and the red dot is the postion of the TurtleBot.

<img src="assets/Implementation/transformed_maze.png" width="100%">

Lastly, by downsampling to the grid size, we made the maze into grids, and we used an array to represent the maze. Publish the maze array, the goal position, and the TurtleBot position to the controller.

<img src="assets/Implementation/downsampled_maze.png" width="100%">

## Planning

We used A* to find the shortest path from the start position to the end position

<img src="assets/Implementation/path_1.png" width="100%">

We preprocessed the path return by A* such that it only contained the turning points.

<img src="assets/Implementation/path_2.png" width="100%">


## Control
At turning points, we calculated the angle difference between the current state (x, y, ɵ) to the desired angle. Current angle with respect to the fixed from is obtained from look_up_transfrom from tf package. The desired angle is calculated from based on the path.
The twist published only contains angular velocity K<sub>z</sub> &Delta;&theta;

&Delta;&theta; = &theta;<sub>d</sub> - &theta;<sub>0</sub>

<img src="assets/Implementation/control_1.png" width="100%">

Between turning points, the main motion was translation. We still took consideration of small angle differences. A proportional control was used.

![equation](assets/Implementation/matrix.png)

<img src="assets/Implementation/control_2.png" width="100%">

Near the goal position, the goal AR tag sometimes was covered by the TurtleBot. We assumed that the goal position could not change any more. Then, we solved it in two ways. One was to estimate the distance and publish the previous twist certain times (i.e. 40 times). The other one was to modify the message received from vision. When the goal AR tag was not seen, assume the position was the same as the previous location.

<img src="assets/Implementation/control_3.png" width="100%">
