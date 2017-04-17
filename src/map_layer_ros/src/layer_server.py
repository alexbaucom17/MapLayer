#!/usr/bin/env python

#ROS wrapper for map layer code

import rospy
from map_layer_ros.msg import observation
import layer
import numpy as np
import matplotlib.pyplot as plt

#global layer variable
myLayer = layer.Layer()
myCount = 0

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I got an observation of %s", data.name)
	obs = {'name':data.name, 'data':np.array([data.x,data.y])}
	myLayer.add_observation(obs)
	ax = plt.subplot(111)
	plt.cla()
	myLayer.plot_layer(ax)
	plt.pause(0.1)
	global myCount
	plt.savefig('imgs/tmp'+str(myCount)+'.png', bbox_inches='tight')
	myCount = myCount + 1
    
def listener():
		
	#start node
	rospy.init_node('map_layer_server')

	#subscribe to observations
	rospy.Subscriber("layer_observations", observation, callback)

	rospy.loginfo("Server ready")

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    listener()
	
