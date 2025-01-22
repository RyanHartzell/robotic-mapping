# Robotic Mapping and Localization
![map-animation](./resources/img/map-animation.gif)
This repository contains material for the Robotic Mapping and Localization course, taught at the Colorado School of Mines Computer Science Department.


## Course Description
In this course, you will learn how to use robot sensors, such as cameras or laser scanners, to create a 3D map of the environment, and locate the robot within the environment. This is commonly known as simultaneous localization and mapping (SLAM). The course will cover both the front-end and back-end SLAM techniques. Front-end algorithms use raw sensor data (such as camera images) to create primitive 3D environment maps and position estimates of the sensor in this map. The front-end outputs are passed into the back-end algorithms, which refine the results by, for example, removing the outliers and fusing local 3D maps to create a consistent map of the entire environment.

We will briefly review optimization, graph theory, linear algebra, and C++ before presenting the main SLAM techniques. We will introduce the Robot Operating System (ROS) to interface sensors, import data, run SLAM algorithms, and send robot control commands. The course will have hands-on homework/projects that require you to code your SLAM algorithm (in C++ or Python), and test it in ROS on real-world data collected by robots. Students will form teams for a final project, which involves implementing a SLAM system. Interested students can test their algorithms on robots (Clearpath Jackal/Husky) at Mines campus, and are supported to further develop and submit their projects as robotic conference/journal papers.


## Prerequisites: 
Knowledge of
- Linear algebra, calculus, probability, and statistics
- Linux, Algorithms, C++, and Python
- Computer vision is helpful, but is not required


## Instructor
Kaveh Fathian, Assistant Professor, Computer Science Department
- Office: Brown 280-N
- Office hours: Tue/Thu 10:45-11:30am (after class)
- Personal website: https://sites.google.com/view/kavehfathian/
- Lab website: https://www.ariarobotics.com/

## References & Acknolegements
The material of this course is collected from severl courses, lectures, and code repositories. The corresponding references are cited at the end of each lecture slide.
