#!/usr/bin/env python

#ROS wrapper for map layer code

import rospy
from map_layer_ros.msg import observation
import layer
import numpy as np
import matplotlib.pyplot as plt

#default values for layer parameters
default_cov_scale = 5.0
default_T_nov = 0.1
default_v_min = 5.0
default_sp_min = 2.5


class layer_instance:
	def __init__(self,layer_name,cfg_dict,plot=False,save_imgs=False):
		"""Create and instance of a layer
				layer_name - string to identify the layer
				cfg_dict - configuration loaded from ros parameter server
				plot - if plots should be displayed, only works if a single layer is created due to limitations with tkinter in multithreaded programs
				save_imgs - to save plotted images for creating animation later """

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


    
def layer_server():
		
	#start node
	rospy.init_node('map_layer_server')

	#load parameters
	layer_names = rospy.get_param("~layer_names")
	layer_name_list = layer_names.split()

	#set up layers
	layer_objects = {}
	for name in layer_name_list:

		#load layer configuration parameters
		layer_cfg = rospy.get_param("~"+name)

		#create layers with given configuration
		layer_objects[name] = layer_instance(name,layer_cfg)	

	rospy.loginfo("Server ready")

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    layer_server()
	
