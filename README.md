# Mobile Robot Path Planning Project

## Project Overview

This project focuses on mobile robot path planning and navigation using the TurtleBot3 Waffle Pi, with an emphasis on implementing and exploring Rapidly Exploring Random Trees (RRT) path planning algorithms.

## Hardware Setup

- **Robot**: TurtleBot3 Waffle Pi
- **Sensors**: 
  - Onboard laser scanner
  - Raspberry Pi processor
- **Environment**: Both Gazebo simulation and real-world testing

## Path Planning Techniques

### 1. Default ROS Navigation Stack
- Utilize `move_base` package
- Explore `turtlebot3_navigation.launch` and `move_base.launch`
- Understand global and local planners
- Integrate with `amcl` and `map_server`

### 2. Custom RRT Global Path Planner
- Develop RRT algorithm as a ROS plugin
- Use `costmap_2d` for obstacle detection
- Replace default global path planner

## Experimental Stages

### Simulation Experiments
- Conduct path planning simulations in Gazebo
- Fine-tune `move_base` parameters
- Optimize navigation performance

### Real Robot Navigation
- Use map from previous project
- Plan and follow paths with four waypoints
- Add unexpected obstacles
- Adapt navigation strategy dynamically

## Key Components

- ROS Packages:
  - `move_base`
  - `costmap_2d`
  - Custom RRT planner
- RVIZ visualization
- Obstacle detection and avoidance

## Objectives

- Implement custom path planning algorithm
- Understand ROS navigation stack
- Develop robust navigation strategies
- Handle dynamic environmental challenges

## Experimental Goals

- Visualize planned and actual robot paths
- Analyze path planning performance
- Demonstrate adaptive navigation capabilities
