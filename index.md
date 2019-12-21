---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: page
title: Introduction

---
<link href="assets/stylesheets/main.css" rel="stylesheet">
# <span class="rainbow"> Welcome to the Maze Runner Bot Website! </span>

This website is meant to document and showcase a final project for the introductory robotics course (EEC106A) at UC Berkeley. This page provides a general summary of the project.

---
# Table of Contents
1. [What are we even doing?](#what-are-we-even-doing)
2. [Why do we even care?](#why-do-we-even-care)
3. [How the heck is this even useful?](#how-the-heck-is-this-even-useful)

---

# What are we even doing?

The original goal of our project was to do some cool pacman thing that involved multiple robots interacting with each other in a three-dimensional maze. The "Pacman" Turtlebot would use feedback from an external camera to explore the maze, while the "ghost" Turtlebots would use SLAM to navigate the level and search for "Pacman". 

<center><img src="assets/introduction/pacman_goals.jpg" width="75%"></center>
*<center><sub><sup>this is how it looked in my head i swear (<a href="https://www.createunsw.com.au/projects/pi-robotic-pacman/">source</a>)</sup></sub></center>*


We broke the problem down into several small milestones,and came up with a timeline to accomplish them. However, once we seriously started working on the project, we realized that our original objective was sort of way too hard for us. In hindsight we were probably a tiny bit too optimistic about the speed in which we would be able to accomplish these milestones, especially since none of us had any experience working on robotics-related projects prior to this class. As a result we scaled the project down to our first milestone: **<u>Having a Turtlebot follow a constrained path to a goal in a two-dimensional maze, using video feedback from an external camera mounted on a tripod.</u>** Additionally, we required that the robot should dynamically respond to changes in the goal and changes in the maze. Finally we wanted the robot to be unaffected by any movement in the external camera (provided the entire maze + robot was still being captured in the view of the camera).

<center><img src="assets/introduction/intro_maze_runner.png" width="75%"></center>
*<center><sub><sup>expectation vs reality</sup></sub></center>*

[(back to top)](#table-of-contents)

# Why do we even care?
[(back to top)](#table-of-contents)

# How the heck is this even useful?
[(back to top)](#table-of-contents)