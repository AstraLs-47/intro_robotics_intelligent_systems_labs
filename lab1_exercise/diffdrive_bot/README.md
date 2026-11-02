**Program:** BSc in Software Engineering

**Course:** Introduction to Robotics and Intelligent Systems

**Lecturer:** Mohammed Sraj (MSc AI)

**Lab Assistant:** Mohammed Sraj (MSc AI)

**Academic year:** 2026

# diffdrive_bot: ROS 2 and Gazebo Lab

This is the second part of Lab 1. It shows the ROS 2 ideas from the turtlesim
tutorials again, this time on a complete, working robot in the Gazebo simulator.

diffdrive_bot is a small two-wheeled robot with a lidar and a camera. In this lab you build it, look at
its model in RViz, and drive it around a simulated room in Gazebo. Then Python nodes drive it for you.
The nodes talk to each other with a message, a service and an action, which are defined in the
lab_interfaces package next to this one.

The whole robot is described in one file, urdf/diffdrive_bot.urdf.xacro.
RViz and Gazebo both read this file, so a change you make there shows up in both.
