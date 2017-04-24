# MapLayer
Probabilistic framework for adding semantic map layers

Based on Incremental Learning of Multivariate Gaussian Mixture Models by Engel and Heinen

## Dependencies
Currently developed with python 2.7.12, numpy 1.11.1, dill 0.2.5, and ROS Indigo

If you don't want to use ROS you can simple take all of the source code files except layer_server.py and layer_test_ros.py as these two are only a ROS wrapper for the rest of the code.

## Installation

If you want this repo to be it's own catkin workspace, simple clone the repository and run catkin_make inside of it

If you want to add this code to an existing catkin workspace, clone the repository and copy the map_layer_ros folder from inside MapLayer/src/ to wherever you keep your catkin packages. If you want to run tests with the map_layer_ros package you will also need to copy MapLayer/data to your catkin_ws/ folder (or change the path to the data folder, see more details below in the Run Tests section).

# Usage

## ROS API

## Run Tests

## Explanation of files



