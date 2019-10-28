<img src="http://moveit.ros.org/assets/images/moveit2_logo_black.png" alt="MoveIt! Logo" width="200"/>

-----

Contributions are being reviewed. Refer to https://github.com/acutronicrobotics/moveit2 for a fork maintained by @AcutronicRobotics that includes the latest changes.

-----


## Roadmap
The MoveIt! Motion Planning Framework **for ROS 2.0**

- [Milestones](#milestones)
- [Overview of MoveIt!](http://moveit.ros.org)
- [Installation Instructions](http://moveit.ros.org/install/)
- [Documentation](http://moveit.ros.org/documentation/)
- [Get Involved](http://moveit.ros.org/documentation/contributing/)

## Milestones
Refer to https://github.com/acutronicrobotics/moveit2#milestones

## Progress

<details><summary>Porting from AcutronicRobotics <a href="https://github.com/AcutronicRobotics/moveit2">repo</a></summary>

| Package name | Package name | Package name                     |        PR opened             | PR merged            | Notes    |
|:------------:|:------------:|:--------------------------------:|:----------------------------:|:--------------------:|:--------:|
|moveit_ros    |              |                                  | :white_medium_square:        | :white_medium_square:|          |
|              | planning     |                                  | :white_medium_square:        | :white_medium_square:|          |
|              |              | robot_model_loader               | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | trajectory_execution_manager     | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | planning_request_adapter_plugins | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | constraint_sampler_manager_loader| :heavy_check_mark:           | :white_medium_square:|          |
|              |              | rdf_loader                       | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | planning_pipeline                | :white_medium_square:        | :white_medium_square:|New commits added after merging - Need Update|
|              |              | plan_execution                   | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | kinematics_plugin_loader         | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | planning_components_tools        | :white_medium_square:        | :white_medium_square:|No progress made for porting|
|              |              | collision_plugin_loader          | :heavy_check_mark:           | :white_medium_square:|          |
|              |              | planning_scene_monitor           | :heavy_check_mark:           | :white_medium_square:|          |
|              |benchmarks    |                                  | :white_medium_square:        | :white_medium_square:|No progress made for porting|
|              |manipulation  |                                  | :white_medium_square:        | :white_medium_square:||
|              |move_group    |                                  | :white_medium_square:        | :white_medium_square:||
|              |perception    |                                  | :white_medium_square:        | :white_medium_square:|No progress made for porting|
|              |planning_interface|                              | :white_medium_square:        | :white_medium_square:||
|              |              | py_bindings_tools        | :white_medium_square:        | :white_medium_square:||
|              |              | common_planning_interface_objects        | :white_medium_square:        | :white_medium_square:||
|              |              | planning_scene_interface        | :white_medium_square:        | :white_medium_square:||
|              |              | move_group_interface        | :white_medium_square:        | :white_medium_square:||
|              |              | robot_interface        | :white_medium_square:        | :white_medium_square:||
|              |              | test        | :white_medium_square:        | :white_medium_square:||
|              |robot_interaction|                               | :white_medium_square:        | :white_medium_square:||
|              |visualization |                                  | :white_medium_square:        | :white_medium_square:||
|              |warehouse     |                                  | :white_medium_square:        | :white_medium_square:||
|moveit_core   |              |                                  | :heavy_check_mark:           | :heavy_check_mark:   ||

</details>

## Continuous Integration Status

[![Build Status](https://travis-ci.org/ros-planning/moveit2.svg?branch=master)](https://travis-ci.org/ros-planning/moveit2)

## Docker Containers

TODO [Create ROS2 Docker containers for MoveIt!](https://github.com/ros-planning/moveit2/issues/15)

## ROS Buildfarm

Debian releases of MoveIt2 will not be available during the alpha development stage. Check back May 2019.
