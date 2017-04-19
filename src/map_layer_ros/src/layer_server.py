#!/usr/bin/env python

#ROS wrapper for map layer code

import rospy
from map_layer_ros.msg import observation
from map_layer_ros.srv import SimpleLookup,SimpleLookupRequest, SimpleLookupResponse
import layer
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import copy


#default values for layer parameters
default_cov_scale = 5.0
default_T_nov = 0.1
default_v_min = 5.0
default_sp_min = 2.5

layer_objects = {}


class layer_instance:
	def __init__(self,layer_name,cfg_dict,plot=False,save_imgs=False,publish=True):
		"""Create an instance of a layer
				layer_name - string to identify the layer
				cfg_dict - configuration loaded from ros parameter server
				plot - if plots should be displayed, only works if a single layer is created due to limitations with tkinter in multithreaded programs
				save_imgs - to save plotted images for creating animation later
				publish - to publish visualization markers on a ros topic """

		#check for proper configuration
		if 'observation_topic' in cfg_dict:
			obs_topic = cfg_dict['observation_topic']
			rospy.loginfo('Layer [%s] - Setting up layer with observation on topic %s',layer_name, obs_topic)
		else:
			rospy.logerr('Layer [%s] - observation_topic is a required parameter',layer_name)
			obs_topic = "dummy_topic"

		if 'covariance_scale' in cfg_dict:
			cov_scale = cfg_dict['covariance_scale']
		else:
			rospy.logwarn('Layer [%s] - covariance_scale being set to default value',layer_name)
			cov_scale = default_cov_scale

		if 'novelty_constant' in cfg_dict:
			T_nov = cfg_dict['novelty_constant']
		else:
			rospy.logwarn('Layer [%s] - novelty_constant being set to default value',layer_name)
			T_nov = default_T_nov

		if 'v_min' in cfg_dict:
			v_min = cfg_dict['v_min']
		else:
			rospy.logwarn('Layer [%s] - v_min being set to default value',layer_name)
			v_min = default_v_min

		if 'sp_min' in cfg_dict:
			sp_min = cfg_dict['sp_min']
		else:
			rospy.logwarn('Layer [%s] - p_min being set to default value',layer_name)
			sp_min = default_sp_min


		#igmm layer
		self.L = layer.Layer(cov_scale,T_nov,v_min,sp_min)

		#observation topic
		self.sub = rospy.Subscriber(obs_topic, observation, self.callback)

		#save info
		self.name = layer_name
		self.cfg_dict = cfg_dict
		self.publish = publish
		
		if self.publish:
			self.marker_pub = rospy.Publisher(self.name+"_markers"	, Marker, queue_size = 100)
			

		#set up plotting stuff
		self.plot = plot
		self.save_imgs = save_imgs
		self.ax = None
		if self.plot and self.save_imgs:
			self.count = 0

	def callback(self,data):

		#print for debugging
		rospy.loginfo("Layer [%s] - got an observation of %s",self.name, data.name)

		#add observation to layer
		obs = {'name':data.name, 'data':np.array([data.x,data.y])}
		self.L.add_observation(obs)
		
		#get data points for vizualization and publish
		if self.publish:
			data = self.L.get_viz_data()
			points_list = [(x,class_data['name']) for class_data in data for x in class_data['means']]
			names = [x[1] for x in points_list]
			points_2d = [x[0] for x in points_list]
			points_3d = to3dPoints(points_2d)
			self.publish_points(points_3d,names)

		#show plot if desired
		if self.plot:

			#create axes if needed
			if not self.ax:
				plt.figure()
				self.ax = plt.subplot(111)

			#plot layer
			self.ax.cla()
			self.L.plot_layer(self.ax)
			plt.pause(0.1)

			#save imges for animation if desired
			if self.save_imgs:
				plt.savefig('imgs/tmp'+str(self.count)+'.png', bbox_inches='tight')
				self.count += 1

	def get_closest(self,class_name,xy):
		return self.L.get_closest(class_name,xy)

	def get_most_likely(self,class_name):
		return self.L.get_most_likely(class_name)
		
	def publish_points(self,point_list,name_list):
		"""Prep marker message with points and publish"""
		
		#get a list of color definitions to map to
		n_colors = 7
		color_defn = [ColorRGBA() for i in xrange(n_colors)]
		#set all alpha values to 1 so they are visible
		for i in xrange(n_colors):
			color_defn[i].a = 1.0
		#red
		color_defn[0].r = 1.0
		#green
		color_defn[1].g = 1.0
		#blue
		color_defn[2].b = 1.0
		#white
		color_defn[3].r = 1.0
		color_defn[3].g = 1.0
		color_defn[3].b = 1.0
		#red+green
		color_defn[4].r = 1.0
		color_defn[4].g = 1.0
		#red+blue
		color_defn[5].r = 1.0
		color_defn[5].b = 1.0
		#green + blue
		color_defn[6].g = 1.0
		color_defn[6].b = 1.0
		
		name_map = {}
		color_list = []
		count = 0		
		#set up and publish name of each point	
		for i in xrange(len(point_list)):
			viz_text = Marker()
			viz_text.header.frame_id = "/map"
			viz_text.header.stamp = rospy.get_rostime()
			viz_text.ns = self.name+"_text"
			viz_text.id = i
		
			viz_text.action = Marker.ADD
			viz_text.type = Marker.TEXT_VIEW_FACING
			viz_text.scale.z = 0.4
			viz_text.color.a = 1.0
			viz_text.color.r = 1.0
			viz_text.color.g = 1.0
			viz_text.color.b = 2.0
			
			viz_text.pose.orientation.w = 1.0
			viz_text.pose.position = copy.deepcopy(point_list[i])
			viz_text.pose.position.z += 0.4 #move text up a little bit
			viz_text.text = "["+self.name+"] "+name_list[i]
			self.marker_pub.publish(viz_text)
			
			#create mapping for this name if we haven't seen it yet
			if name_list[i] not in name_map:
				name_map[name_list[i]] = count%n_colors #make colors wrap if there aren't enough
				count += 1
			
			#create list of colors to corespond to class names
			color_list.append(color_defn[name_map[name_list[i]]])
			
			#end for	
		
		#publish the actual points	
		viz_points = Marker()
		viz_points.header.frame_id = "/map"
		viz_points.header.stamp = rospy.get_rostime()
		viz_points.ns = self.name+"_points"
		
		viz_points.action = Marker.ADD
		viz_points.type = Marker.POINTS
		viz_points.pose.orientation.w = 1.0
		
		viz_points.scale.x = 0.2
		viz_points.scale.y = 0.2
		viz_points.colors = color_list 
		
		viz_points.points = point_list
		self.marker_pub.publish(viz_points)
			
		

def to3dPoints(points_2d):
	"""Simple function to take list of 2d coordinates and output list of 3d ros points"""

	points_3d = []
	for (x,y) in points_2d:
		p = Point() 
		p.x = x
		p.y = y
		p.z = 0
		points_3d.append(p)    
	return points_3d
	

def SimpleLookupCallback(req):
	"""Perform simple lookup service
		req has 4 fields: layer_name, class_name, modifier, and possibly a pose
		This service is expected to return an xy location and possibly a class name"""
		

	if req.modifier == "most_likely":
		xy = layer_objects[req.layer_name].get_most_likely(req.class_name)
		return {'x':xy[0], 'y':xy[1]} #using a dict populates the correct response field for the service
		
	elif req.modifier == "closest":
	    xy_pose = np.array([req.pose.x,req.pose.y])
	    ret_dict = layer_objects[req.layer_name].get_closest(req.class_name,xy_pose)
	    return {'class_name':ret_dict['class_name'],'x':ret_dict['xy'][0], 'y':ret_dict['xy'][1]}
	    
	else:
		rospy.logerr('Unknown modifier request')
		return None		
			


    
def layer_server():
		
	#start node
	rospy.init_node('map_layer_server')

	#load parameters
	layer_names = rospy.get_param("~layer_names")
	layer_name_list = layer_names.split()

	#set up layers
	global layer_objects
	for name in layer_name_list:

		#load layer configuration parameters
		layer_cfg = rospy.get_param("~"+name)

		#create layers with given configuration
		layer_objects[name] = layer_instance(name,layer_cfg)	
		
	#set up service hanlder
	s = rospy.Service("map_layer_simple_lookup",SimpleLookup,SimpleLookupCallback)
	rospy.loginfo("Map Layer Server ready")

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    layer_server()
	
