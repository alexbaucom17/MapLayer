#Map Layer skeleton code

import numpy as np
#import dill

#TODO: Verify this can be serialized

class GaussianModel:
    """Class to hold a single gaussian model of a class of observations"""

    def __init__(self,mu0,sig0):
        """Initialize Gaussian model with intial mean and covariance
            mu0 should be nx1
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
       
        # add new observation to list and increase array size if needed
        if self.obs_count_ <= self.obs_data_.shape[1]:
            self.obs_data_[:,self.obs_count_] = observation
            self.obs_count_ += 1
        else:
            self.obs_data_ = np.concatenate((self.obs_data_,np.zeros((self.n_,1000))),axis = 0)
            self.obs_data_[:,self.obs_count_] = observation
            self.obs_count_ += 1
            
        #compute new mean and covariance
        self.mu_ = np.mean(self.obs_data_[:,:self.obs_count_],axis=1)
        self.sig_ = np.cov(self.obs_data_[:,:self.obs_count_],rowvar=False)

    def get_mean(self):
        return self.mu_
        
    def get_cov(self):
        return self.sig_
        
       
if __name__ == "__main__":
        
    #some quick testing
    mu = np.array([[0,0,0]]).T
    sig = 0.5 * np.identity(3)
    GM = GaussianModel(mu,sig)
    print GM.get_mean()
    print GM.get_cov()
    GM.update(np.array([1,2,3]))
    print GM.get_mean()
    print GM.get_cov()
    
        
