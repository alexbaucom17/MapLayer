
import numpy as np

class GaussianModel:
    """Class to hold a single gaussian model of a class of observations"""

    def __init__(self,mu0,sig0):
        """Initialize Gaussian model with intial mean and covariance
            mu0 should be n vector
            sig0 should be nxn"""
        
        self.mu = np.asarray(mu0)   # model mean
        self.sig = np.asarray(sig0) # model covariance
        self.n = self.mu.shape[0]  # model size
        self.obs_data = np.zeros((self.n,1000)) #hold observation data
        self.obs_count = 0          # count how many observations we have seen
        
        #Quick size check
        if not(self.sig.shape[0] == self.n and self.sig.shape[1] == self.n):
            raise ValueError("The shape of mu and sigma do not match")
        
    def update(self,observation):
        """Add a new observation to the model and update the parameters
        observation should be nx1"""

        #size check
        if not observation.shape[0] == self.n:
            raise ValueError("Observation is not the correct size")

        # add new observation to list and increase array size if needed
        if self.obs_count <= self.obs_data.shape[1]:
            self.obs_data[:,self.obs_count] = observation
            self.obs_count += 1
        else:
            self.obs_data = np.concatenate((self.obs_data,np.zeros((self.n,1000))),axis = 0)
            self.obs_data[:,self.obs_count] = observation
            self.obs_count += 1
            
        #compute new mean and covariance, only update sig when we have enough data
        self.mu = np.mean(self.obs_data[:,:self.obs_count],axis=1)
        if self.obs_count > self.n:
            self.sig = np.cov(self.obs_data[:,:self.obs_count])

    def get_mean(self):
        return self.mu
        
    def get_cov(self):
        return self.sig




class SimpleLayer:
    """Simple layer to test out how stuff could work together"""

    def __init__(self,cfg):
        """cfg should be dictionary with config keys"""

        self.cfg = cfg

        #limit config values for now
        if not (self.cfg["type"] == "static" and self.cfg["model"] == "single"):
            raise ValueError("Please choose supported configuration")

        #keep list of known classes
        self.class_list = {}

    def add_observation(self,obs):

        if obs["name"] in self.class_list:
            self.class_list[obs["name"]].update(obs["data"]) #TODO: Figure out how measurement covariance plays in here
        else:
            n = obs["data"].shape[0]
            mu0 = np.zeros(n)
            sig0 = np.identity(n)
            self.class_list[obs["name"]] = GaussianModel(mu0,sig0)
            self.class_list[obs["name"]].update(obs["data"])


    def get_params(self,name):
        if name in self.class_list:
            return (self.class_list[name].get_mean(),self.class_list[name].get_cov())
        else:
            return None

        
