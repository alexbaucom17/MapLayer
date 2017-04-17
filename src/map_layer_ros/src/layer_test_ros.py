#!/usr/bin/env python

import rospy
from map_layer_ros.msg import observation
import GenerateData
import numpy as np


#DATA_FILE = "../../../data/RegionLocations.csv"
DATA_FILE = "../../../data/ObjectLocations.csv"


def talker():

	#initialize node and publishcer
	pub = rospy.Publisher('layer_observations', observation, queue_size=10)
	rospy.init_node('talker')


	# load data from file
	data = GenerateData.load_csv(DATA_FILE)
	# add some noise
	c = 0.3 * np.identity(2)
	noisy_data = GenerateData.noisy_observations(data, 10, c, True)

	#loop
	rate = rospy.Rate(3)
	count = 0 
	n = len(noisy_data)
	while not rospy.is_shutdown():
        
		if count < n:
			obs = observation()
			obs.name = noisy_data[count]['name']
			obs.x = noisy_data[count]['data'][0]
			obs.y = noisy_data[count]['data'][1]
			pub.publish(obs)
			rospy.loginfo("Data published %i", count)
			count += 1
		else:
			rospy.loginfo("Out of data to publish")

		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
