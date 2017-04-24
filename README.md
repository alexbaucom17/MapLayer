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
Set parameters in map_layer_ros/params/layer_params.yaml (or edit the launch file to have a different parameter file). An example file is shown:
```
layer_names: regions objects

#Saving and loading not implimented yet
#layer_file: /home/alex/MapLayer/data/test_layer_save.pickle

regions:
  observation_topic: region_obs
  covariance_scale: 3.0
  novelty_constant: 0.2
  v_min: 6.0
  sp_min: 2.5

objects:
  observation_topic: object_obs
  covariance_scale: 1.0
  novelty_constant: 0.2
  v_min: 6.0
  sp_min: 2.0
  ```

Parameters:
- layer_name: must be a space delimited list of strings that are the names of each layer. Each layer name here must have a block defined below as well
- layer_file: where to save and load data to. Not in use yet.
- For each layer:
    - observation_topic: string that defines the topic that observations will come in on
    - covariance_scale: The scale to use for initializing a new mixture component
    - novelty_constant: Must be greater than 0 and less than or equal to 1. Defines novelty threshold for adding a new mixture component. A value of 1 means add a new component for every observation and values close to 0 mean don't ever add new components. This will need to be tuned for the expected noise and observation spacing for each layer
    - v_min: How many updates an IGMM model must have before checking if a mixture component is relevant. Adjust this based on how often observations come in and how frequently they should be assigned to existing clusters.
    - sp_min: The minimum value of a running probability sum required after v_min updates in order for a cluster to be considered useful. Each component added to a cluster will have a probability of approximately 1, unless there is a lot of overlap with other clusters so a value of 3.0 would likely need 3-4 observations added to that cluseter to qualify. Larger values of v_min and smaller values of sp_min will keep more clusters, whereas small values of v_min and large values of sp_min will keep fewer clusters. Note that when a cluster is removed the points that were previously assigned to it are not updated and integrated into a different cluster as it is assumed that the removed cluster was spurious and therefore not relevant to the performance of the algorithm.


Launch the server with `roslaunch map_layer_ros layer_server.launch`

Send messages of type map_layer_ros.msg/observation on the topics specified in the parameter file. The x and y coordinates of the observation must be in the /map frame.

Vizualize by setting rviz to show markers on the topic `[Layer_name]_markers`. This will show the location of the cluster centers as well as the layer name and the class name of that cluster. Cluster markers are also colored to be distinguishable (at least until there are over 7 classes and then the colors start repeating).


## Run Tests

## Explanation of files



