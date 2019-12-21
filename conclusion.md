---
layout: page
title: Conclusion
permalink: /conclusion
---
# How well did your finished solution meet your design criteria?

* Original goals
    * Have robots reliably actuate and interact with each other in a dynamic environment
    * Implement an accurate vision and path-planning algorithm given a top-down view of the environment
    * Use SLAM to search for a target in a complicated maze

* What we achieved
    * The TurtleBot can move to the goal position from a random position and orientation in a maze.
    * Camera that provides the vision for the TurtleBot does not require to be on top of the maze, also can be at an angled position
    * TurtleBot can find its goal correctly even if we change the goal after it starts moving

After actually starting doing the project, we realized that we won't have enough time to add another TurtleBot. However, we achieved that using angled camera providing information instead of providing top down view, and we also successfully implemented A* search algorithm and controller for the TurtleBot.

# Particular difficulties

1. We found that transform the image using one camera is difficult because the TurtleBot might cover part of the maze/corner causing some issues.

    Solution: We decided that the initial maze will be memorized, and the part TurtleBot covered will be uncertain. If the camera sees a position as a wall/hallway but in previously memorized grid was uncertain, it will store that position as wall. If the camera sees a position as uncertain but in previously memorized grid was a wall or a hallway, it will not change previously memorized grid.

2. We also noticed that sometimes base_link flip randomly.

[![](http://img.youtube.com/vi/SF1i_vUWl2g/0.jpg)](http://www.youtube.com/watch?v=SF1i_vUWl2g "Base_link flipping randomly")

Not entirely sure why this was happening, but sometimes adjusting the AR tag position prevented this happening.

# Improvements if we had additional time

1. Introducing another TurtleBot as an adversary to the first TurtleBot.
2. Making the dynamic controller more robust  
3. Make corners more robust to lighting changes using some adaptive contrast algorithm.
4. Make pixel of TurtleBot (base link) position robust to glitches in ar_track_alvar.
5. Make maze publishing accurate even if TurtleBot covers parts of the maze for a period of time.
6. Control using feedback based on boundary points of the robot's base instead of center.
