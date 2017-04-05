import csv
import numpy as np
import random


DATA_FILE = "../data/ObjectLocations.csv"

def load_csv(fname):
	data = []
	with open(fname,'rb') as csvfile:
		reader = csv.reader(csvfile,delimiter=',')
		for row in reader:
			name = row[0]
			x = float(row[1])
			y = float(row[2])
			obs = {'class':name,'xy':np.array([x,y])}
			data.append(obs)
	return data


def noisy_observations(data_in, avg_obs_per_point, cov, shuffle=False):
	"""Generate random observations from 'true data'
		data - a list of data from load_csv()
		avg_obs_per_point - average number of observations to generate from each 'true' data point
		cov - covariance matrix for drawing random xy data sample (2x2)
		returns list with same structure as load_csv, but with noisy observations"""

	#init	
	data_out = []
	n_in = len(data_in)
	idx = []

	#generate indeces, set 1 to give exact indeces back
	if avg_obs_per_point == 1:
		idx = np.arange(n_in).tolist()
	else:
		for i in xrange(n_in):
			n = int(np.random.normal(avg_obs_per_point))
			idx += [i for j in xrange(n)]

	#shuffle list if desired
	if shuffle:
		random.shuffle(idx) #operates in place

	#grab each data point and draw random sample from N(xy,cov)
	for i in idx:
		data_out.append({'class':data_in[i]['class'],
						 'xy':np.random.multivariate_normal(data_in[i]['xy'],cov)})
	return data_out
	
	


def main():

	#load data from file
	data = load_csv(DATA_FILE)

	#add some noise
	c = 0.5 * np.identity(2)
	noisy_data = noisy_observations(data, 3, c, True)
	
	#print data
	print 'Original Data:'
	for row in data:
		print row
	print 'Noisy Data:'
	for row in noisy_data:
		print row


if __name__ == "__main__":
	main()
