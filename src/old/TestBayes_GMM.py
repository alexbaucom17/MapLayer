#!/usr/bin/env python

import logging
import matplotlib.pyplot as plt
import numpy as np
import random
import sys
import src.GenerateData

sys.path.append("bayes_gmm")

from bayes_gmm.niw import NIW
from bayes_gmm.igmm import IGMM
from bayes_gmm.plot_utils import plot_ellipse, plot_mixture_model

logging.basicConfig(level=logging.INFO)

random.seed(1)
np.random.seed(1)
DATA_FILE = "../data/ObjectLocations.csv"


def data2matrix(data):
	n = len(data)
	mat = np.zeros((n,2))
	for i in xrange(n):
		mat[i,:] = data[i]['xy']
	return mat

def main():


	#load data from file
	data = src.GenerateData.load_csv(DATA_FILE)

	#add some noise
	c = 0.5 * np.identity(2)
	noisy_data = src.GenerateData.noisy_observations(data, 3, c, True)
	mat = data2matrix(noisy_data)

	# Model parameters
	alpha = 1
	K = 3           # initial number of components
	n_iter = 20
	D = 2 #dimension

	# Intialize prior
	mu_scale = 4.0
	covar_scale = 0.7
	m_0 = np.zeros(D)
	k_0 = covar_scale**2/mu_scale**2
	v_0 = D + 3
	S_0 = covar_scale**2*v_0*np.eye(D)
	prior = NIW(m_0, k_0, v_0, S_0)

	for i in xrange(1,mat.shape[0]):

		#create igmm
		if i == 1:
			A = np.array([-1])
		else:
			A = np.concatenate((A,np.array([-1])))
		igmm = IGMM(mat[:i].reshape(i,2), prior, alpha, assignments=A, K=K)

		# Perform Gibbs sampling
		record = igmm.gibbs_sample(n_iter)

		A = igmm.components.assignments

		# Plot results
		fig = plt.figure()
		ax = fig.add_subplot(111)
		plot_mixture_model(ax, igmm)
		for k in xrange(igmm.components.K):
			mu, sigma = igmm.components.rand_k(k)
			plot_ellipse(ax, mu, sigma)
		plt.show()


if __name__ == "__main__":
    main()
