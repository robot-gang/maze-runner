---
layout: page
title: Design
permalink: /design
---

# What design criteria must our project meet? What is the desired functionality?

* # Vision

* # Planning and Control

The planner should be able to find the shortest path from the starting position to the goal position. To be robust, a planner must correctly handle cases like no path and multiple paths. The controller brings the TurtleBot to the desired position while following the shortest path returned by a planner.  We planned to create two controllers. One is a fixed path controller, which assumes that the environment of the TurtleBot stays invariant. The planner will compute the shortest path the first time when it receives a valid TurtleBot maze configuration, and the system will memorize the path. Another one is a dynamic path controller, which assumes that the environment and the goal position are changeable. To cope with the change, the controller requires the planner to update the path whenever it receives a TurtleBot maze configuration.

# Design we choose

We used A* algorithm to compute the shortest path. There were two reasons to choose A*. First, A* is deterministic and optimal if the heuristic is consistent and admissible. Second, the computation is very fast when the TurtleBot maze configuration is small.  The heuristic was euclidean distance because diagonal motion was allowed.
The core of control was a proportional controller. At each turning point of the path, the desired direction changed. Hence, turning to the desired angle prohibited the error accumulation.

# What design choices did we make when formulated our design?  What trade-oﬀs did we have to make?

After computing the shortest path, we first removed all the non-turning points. The motivation to develop a fixed path controller was to split the path into a sequence of shorter routes. Each route had two turning points, and the motion along that route was a pure rotation and pure translation. The pure rotation at the turning point had two main functionalities. One was to align the TurtleBot to the desired orientation, and the other was for error correction. We chose the goal frame as the fixed frame so that the look_up_transofrm function from tf package could return the orientation to the goal frame. For each route, we computed the desired angle in the fixed frame, then we used proportional control to correct the orientation of the TurtleBot. In dynamic path control, the main difference was that we updated the maze TurtleBot configuration and the corresponding path frequently. The challenge to adapt the fixed path controller to the dynamic path controller was that TurtleBot started turning too early. Due to the way how we calculated the grid location, the TurtleBot started turning once the center of it entered a grid, not necessarily at the center of the grid. To solve it, we defined a threshold near the center of a grid (turning point) so that the state (x, y, orientation) only got updated when the center of the TurtleBot was close to the center of the grid. The problem induced was that if the TurtleBot was not close enough to the center of a turning point, it still kept the state of the previous turning point. To further solve the dynamic control, we needed to implement a more sophisticated state update rule.


# How do these design choices impact how well the project meets design criteria that would be encountered in a real engineering application, such as robustness, durability, and eﬃciency?

The fixed path controller worked very well because our assumption oversimplified the problem. One can treat the fixed path controller the same as how we figure out the path from home to school without a GPS. Without any information update, we choose the shortest/fastest path from a memorized map. It performs as good as how we can find the lab every day. There were two reasons we might want to work on dynamic control. First, the goal position might change. As an example, one might change his mind on the way to Moffitt library and decide to study at the Engineering library. Second, the path or the environment might change because some road construction might block your way to the Engineering library. Therefore, the TurtleBot also needed to adjust its path frequently. Due to the simplified state update rule, the dynamic controller was not robust. When we changed the goal or maze, the TurtleBot worked best at a turning point, it worked okay near the center of a grid, and it barely worked near the edges of grids because it was confused about its location.
