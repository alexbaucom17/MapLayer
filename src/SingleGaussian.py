

import numpy as np

class GaussianModel:
    """Class to hold a single gaussian model of a class of observations"""

    def __init__(self,mu0,sig0):
        """Initialize Gaussian model with intial mean and covariance
            mu0 should be n vector
            sig0 should be nxn"""
        
        self.mu_ = np.asarray(mu0)   # model mean
        self.sig_ = np.asarray(sig0) # model covariance
        self.n_ = self.mu_.shape[0]  # model size
        self.obs_data_ = np.zeros((self.n_,1000)) #hold observation data
        self.obs_count_ = 0          # count how many observations we have seen
        
        #Quick size check
        if not(self.sig_.shape[0] == self.n_ and self.sig_.shape[1] == self.n_):
            raise ValueError("The shape of mu and sigma do not match")
        
    def update(self,observation):
        """Add a new observation to the model and update the parameters
        observation should be nx1"""

        #size check
        if not observation.shape[0] == self.n_:
            raise ValueError("Observation is not the correct size")

        # add new observation to list and increase array size if needed
        if self.obs_count_ <= self.obs_data_.shape[1]:
            self.obs_data_[:,self.obs_count_] = observation
            self.obs_count_ += 1
        else:
            self.obs_data_ = np.concatenate((self.obs_data_,np.zeros((self.n_,1000))),axis = 0)
            self.obs_data_[:,self.obs_count_] = observation
            self.obs_count_ += 1
            
        #compute new mean and covariance, only update sig when we have enough data
        self.mu_ = np.mean(self.obs_data_[:,:self.obs_count_],axis=1)
        if self.obs_count_ > self.n_:
            self.sig_ = np.cov(self.obs_data_[:,:self.obs_count_])

    def get_mean(self):
        return self.mu_
        
    def get_cov(self):
        return self.sig_
        

        



    
        
