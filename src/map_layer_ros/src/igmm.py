#!/usr/bin/env python

import numpy as np
import GenerateData
import plot_utils
import matplotlib.pyplot as plt


class IGMM:
    """Incremental Gaussian Mixture Model Class
    
    Based on "Incremental Learning of Multivariate Gaussian Mixture Models" by Engel and Heinen
    """

    def __init__(self,n,sig_init,T_nov,v_min=5.0,sp_min=2.5):
        """Initialize igmm
            
            Parameters:
                n - dimension of data
                sig_init - scale for initial covariance matrix
                T_nov - novelty constant 0 < T_nov <= 1, defines distance that new data point must be from
                    any other components in order to create a new component. Bigger means that more components will be
                    created and T_nov = 1 means that every point will have a new component. The authors used 0.01             
				v_min - how many updates must pass before checking if a component should be removed
				sp_min - minimum cumulative probability to keep a component
        """

        self.n_dim = n
        self.n_comp = 0
        self.sig_init = sig_init
        self.T_nov = T_nov
        self.v_min = v_min
        self.sp_min = sp_min


    def update(self,X):
        """Update igmm with new data point
            X - data point that should be n length numpy array
        """

        #size check
        if X.shape[0] != self.n_dim:
            raise ValueError("The length of the input vector must match the dimension of the mixture model")

        #initialize variables if this is the first data point
        if self.n_comp == 0:

            self.n_comp = 1
            self.mu = [X] #component mean
            self.sig = [self.sig_init**2*np.identity(self.n_dim)] #component covariance
            self.alpha = [1.0] #component weight/prior, called p(j) in the paper
            self.v = [1.0] #age of component j
            self.sp = [1.0] #accumulator of component j

        else:

            # compute the probabilty of belonging to each component and the threshold for that component
            p_x_given_j = np.zeros(self.n_comp)
            thresh = np.zeros(self.n_comp)
            for j in xrange(self.n_comp):
                # get current component parameters
                Cj = self.sig[j]
                uj = self.mu[j]

                # compute probability
                a = (2 * np.pi) ** (self.n_dim / 2.0) * np.sqrt(np.linalg.norm(Cj))
                xu = (X - uj).reshape(-1, 1)
                b = -0.5 * np.dot(xu.T, np.dot(np.linalg.inv(Cj), xu))
                p_x_given_j[j] = 1 / a * np.exp(b)
                thresh[j] = self.T_nov / a


            #if we meet novelty criterion, then add new component
            if np.all(np.less(p_x_given_j,thresh)):

                self.n_comp += 1
                self.mu.append(X)  # component mean
                self.sig.append(self.sig_init ** 2 * np.identity(self.n_dim))  # component covariance
                self.v.append(1)  # age of component j
                self.sp.append(1)  # accumulator of component j
                self.alpha.append(1/sum(self.sp))  # component weight/prior, called p(j) in the paper

            else:
                # otherwise we update all the components with the new data

                #use bayes rule to find p_j_given_x
                sum_pj = sum([p_x_given_j[j] * self.alpha[j] for j in xrange(self.n_comp)])
                p_j_given_x = [p_x_given_j[j] * self.alpha[j]/sum_pj for j in xrange(self.n_comp)]

                #do updates exactly as specified in the paper
                for j in xrange(self.n_comp):
                    self.v[j] += 1
                    self.sp[j] += p_j_given_x[j]
                    ej = X - self.mu[j]
                    wj = p_j_given_x[j]/self.sp[j]
                    d_uj = wj * ej
                    self.mu[j] += d_uj
                    ej_star = X - self.mu[j]
                    self.sig[j] = (1-wj)*self.sig[j] + wj*np.outer(ej_star,ej_star) - np.outer(d_uj,d_uj)

            # check if any components need to be removed
            idx2remove = [j for j in xrange(self.n_comp) if self.v[j] > self.v_min and self.sp[j] < self.sp_min]
            for j in sorted(idx2remove, reverse=True):
                self.n_comp -= 1
                del self.mu[j]
                del self.sig[j]
                del self.v[j]
                del self.sp[j]
                del self.alpha[j]

            #normalize alpha values after all sp updates
            sum_sp = float(sum(self.sp))
            self.alpha = [self.sp[j]/sum_sp for j in xrange(self.n_comp)]


    def get_most_likely(self):
        return self.mu[np.argmax(self.sp)]
        
    def get_closest(self,xy):
        best_dist = float("Inf")
        best_mu = None
        for mu in self.mu:
            dist = np.linalg.norm(xy-mu)
            if  dist < best_dist:
                best_dist = dist
                best_mu = mu
        return best_mu

    def get_means(self):
        return self.mu
		
    def get_covs(self):
        return self.sig

    def plot(self,ax,color='b'):

        #plot each component
        for j in xrange(self.n_comp):
            plot_utils.plot_ellipse(ax, self.mu[j], self.sig[j],color)



DATA_FILE = "../data/RegionLocations.csv"
#DATA_FILE = "../data/ObjectLocations.csv"

if __name__ == "__main__":

    #initialize gmm
    gmm = IGMM(2, 6, 0.1)  #for region locations
    #gmm = IGMM(2, 3, 0.1)  # for object locations

    # load data from file
    data = GenerateData.load_csv(DATA_FILE)
    # add some noise
    c = 0.4 * np.identity(2)
    noisy_data = GenerateData.noisy_observations(data, 10, c, True)

    #add data points
    X = []
    for pt in noisy_data:
        gmm.update(pt['data'])

        #put point into X for eas plotting
        X.append(pt['data'])

    #plot
    ax = plt.subplot(111)
    X = np.asarray(X)
    ax.scatter(X[:, 0], X[:, 1], s=10)
    gmm.plot(ax)
    plt.show()
