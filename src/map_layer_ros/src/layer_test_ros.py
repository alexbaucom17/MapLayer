#!/usr/bin/env python

import rospy
from map_layer_ros.msg import observation
import GenerateData
import numpy as np


DATA_FILE1 = "../../../data/MapBasedRegionLocations.csv"
DATA_FILE2 = "../../../data/MapBasedObjectLocations.csv"


def talker():
	"""Sets up two data streams and publishes semi-random noisy data for testing map layer code"""

	#initialize node and publishcer
	pub2 = rospy.Publisher('object_obs', observation, queue_size=10)
	pub1 = rospy.Publisher('region_obs', observation, queue_size=10)
	rospy.init_node('talker')


	# load data from region file
	data1 = GenerateData.load_csv(DATA_FILE1)
	# add some noise
	c = 0.6 * np.identity(2)
	noisy_data1 = GenerateData.noisy_observations(data1, 5, c, True)

	# load data from object file
	data2 = GenerateData.load_csv(DATA_FILE2)
	# add some noise
	c = 0.3 * np.identity(2)
	noisy_data2 = GenerateData.noisy_observations(data2, 5, c, True)

	#loop
	rate = rospy.Rate(5)
	count1 = 0 
	count2 = 0
	n1 = len(noisy_data1)
	n2 = len(noisy_data2)
	while not rospy.is_shutdown():

		#hacky way to have semi-random input stream with multiple layers
		if np.random.uniform() > 0.5:
		    
			#create observation on one topic
			if count1 < n1:
				obs = observation()
				obs.name = noisy_data1[count1]['name']
				obs.x = noisy_data1[count1]['data'][0]
				obs.y = noisy_data1[count1]['data'][1]
				pub1.publish(obs)
				rospy.loginfo("[Data1] published %i", count1)
				count1 += 1
			else:
				rospy.loginfo("[Data1] Out of data to publish")

		else:

			#create observation on a different topic
			if count2 < n2:
				obs = observation()
				obs.name = noisy_data2[count2]['name']
				obs.x = noisy_data2[count2]['data'][0]
				obs.y = noisy_data2[count2]['data'][1]
				pub2.publish(obs)
				rospy.loginfo("[Data2] published %i", count2)
				count2 += 1
			else:
				rospy.loginfo("[Data2] Out of data to publish")
		

		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
