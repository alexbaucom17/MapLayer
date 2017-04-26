# MapLayer
This is a probabilistic framework for adding semantic map layers. If you have a stream of observations with a location and a name (maybe from an object detector or scene recognizer), this package can cluster those nosiy observations and keep them organized in layers for future reference or lookup.

The ROS wrapper for this code restricts the observations to be 2D but the underlying code can handle any dimensionality of observation and even have layers where each class has a different dimensionality.

Based on 

Engel, Paulo Martins, and Milton Roberto Heinen. "Incremental learning of multivariate gaussian mixture models." Brazilian Symposium on Artificial Intelligence. Springer Berlin Heidelberg, 2010.

and 

Pinto, Rafael Coimbra, and Paulo Martins Engel. "A fast incremental gaussian mixture model." PloS one 10.10 (2015): e0139931.

The first paper outlines the algorithm but the second paper presents a much more concise and clear set of equations for the algorithm. We did not impliment the 'fast' version of the algorithm since the data we were working with was only 2D.

## Dependencies
Currently developed with python 2.7.12, numpy 1.11.1, dill 0.2.5, and ROS Indigo.

If you don't want to use ROS you can simple take all of the source code files except layer_server.py and layer_test_ros.py as these two are only a ROS wrapper for the rest of the code. There is no dimensionality restriction on the base code wheras the ROS version is restriced to 2D because of the observation.msg type.

## Installation

If you want this repo to be it's own catkin workspace, simple clone the repository and run catkin_make inside of it. Make sure to source the package as well with `source devel/setup.bash`

If you want to add this code to an existing catkin workspace, clone the repository and copy the map_layer_ros folder from inside MapLayer/src/ to wherever you keep your catkin packages. If you want to run tests with the map_layer_ros package you will also need to copy MapLayer/data to your catkin_ws/ folder (or change the path to the data folder, see more details below in the Run Tests section). Make sure to rebuild your workspace and source it as well.

# Usage

## ROS API
Set parameters in map_layer_ros/params/layer_params.yaml (or edit the launch file to have a different parameter file). An example file is shown:
```
layer_names: regions objects

layer_file: /home/alex/MapLayer/data/test_layer_save.pickle

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
- layer_file: where to save and load data to. Comment out to ignore loading and saving. If new layers are specified in the parameter file that were not saved in the file, they will be created new. Old layers that were saved will not re-load parameters. A file will automatically be saved here when the service is killed.
- For each layer:
    - observation_topic: string that defines the topic that observations will come in on
    - covariance_scale: The scale to use for initializing a new mixture component
    - novelty_constant: Must be greater than 0 and less than or equal to 1. Defines novelty threshold for adding a new mixture component. A value of 1 means add a new component for every observation and values close to 0 mean don't ever add new components. This will need to be tuned for the expected noise and observation spacing for each layer
    - v_min: How many updates an IGMM model must have before checking if a mixture component is relevant. Adjust this based on how often observations come in and how frequently they should be assigned to existing clusters.
    - sp_min: The minimum value of a running probability sum required after v_min updates in order for a cluster to be considered useful. Each component added to a cluster will have a probability of approximately 1, unless there is a lot of overlap with other clusters so a value of 3.0 would likely need 3-4 observations added to that cluseter to qualify. Larger values of v_min and smaller values of sp_min will keep more clusters, whereas small values of v_min and large values of sp_min will keep fewer clusters. Note that when a cluster is removed the points that were previously assigned to it are not updated and integrated into a different cluster as it is assumed that the removed cluster was spurious and therefore not relevant to the performance of the algorithm.


Launch the server with `roslaunch map_layer_ros layer_server.launch`

Send messages of type map_layer_ros.msg/observation on the topics specified in the parameter file. The x and y coordinates of the observation must be in the /map frame.

Vizualize by setting rviz to show markers on the topic `[Layer_name]_markers`. This will show the location of the cluster centers as well as the layer name and the class name of that cluster. Cluster markers are also colored to be distinguishable (at least until there are over 7 classes and then the colors start repeating).

## Lookup Requests
The layer server also provides a framework for ROS service based lookup requests. There are currently two supported lookup requests that use the same service called SimpleLookup.

The first will take a layer name and a class name and return the most likely location of that class. The second will take a layer name, a class name (or 'any'), and a pose and return the closest location of the speicifed class (or of any class) to the given pose.

The server must have recieved some observations for the speicfied layer and class before these service calls will do anything useful.

Some sample service calls are shown below:
```(python)
from map_layer_ros.srv import SimpleLookup, SimpleLookupRequest, SimpleLookupResponse

#setup service proxy
rospy.wait_for_service('map_layer_simple_lookup')
lookup = rospy.ServiceProxy('map_layer_simple_lookup', SimpleLookup)  

#request the most likely location of the class 'chair' in the 'objects' layer
req = SimpleLookupRequest()
req.modifier = 'most_likely'
req.layer_name = 'objects'
req.class_name = 'chair'
try:
  resp1 = lookup(req) #resp1 will have fields x,y,class_name
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))
 
#request the closest location of the class 'office' in the 'regions' layer
req = SimpleLookupRequest()
req.modifier = 'closest'
req.layer_name = 'regions'
req.class_name = 'office'
req.pose = my_current_pose
try:
  resp2 = lookup(req) #resp2 will have fields x,y,class_name
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))
  
#request the closest location of any object to the centroid of the 'office' class we previously looked up
req = SimpleLookupRequest()
req.modifier = 'closest'
req.layer_name = 'objects'
req.class_name = 'any'
req.pose = xyToPose(resp2.x,resp2.y)
try:
  resp3 = lookup(req) #resp3 will have fields x,y,class_name
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))
```

If you wish to add more lookup functionality or modify existing functionality, the current service runs in the SimpleLookupCallback() function in layer_server.py. It is also fairly straightforward to add other services. If you need help setting up a service, here are [some instructions](http://wiki.ros.org/rospy/Overview/Services).


## Run Tests

To verify that everything is working properly you can use `rosrun map_layer_ros layer_test_ros.py` which will run a script to publish some fake data and test that the map_server is working. If you get errors reading the data file, you might need to change the top lines of layer_test_ros.py to give the correct path to the data. Currently, the path is set to work properly if the script is run while MapLayer/src/map_layer_ros/src is the current directory.

If you are not using the ROS versions of the code, you can use `python layer.py` to test the code. Again, the path to the data might need to be adjusted and you also might need to set the self.debug flag in layer.__init__() to True in order to get the plot to work properly.

## Explanation of source files
All of these files can be found in map_layer_ros/src/
- igmm.py - the core file defining the igmm class. Based almost exactly on Incremental Learning of Multivariate Gaussian Mixture Models by Engel and Heinen. 
- layer.py - defines the layer class which is a collection of igmm classes based on the label name from the observations. If a new label is seen, a new igmm model is created. If a known name is seen, the igmm associated with that layer is updated.
- layer_server.py - a ROS wrapper for the layer class. Creates as many layers as specified by a ros parameter file and sets up subscriptions to the specified observation topics. Advertises ROS services for lookup requests.
- layer_test_ros.py - Publishes generated data on ROS topics for easily testing the layer_server code
- GenerateData.py - defines a few functions to quickly generate noisy data and multiple observations from .csv files which list the 'true' data.
- plot_utils.py - utility for plotting mixture models with matplotlib



