
import numpy as np
import igmm
import GenerateData
import matplotlib.pyplot as plt

class Layer:
    """Class that holds a layer of the map represented as several igmms"""

    def __init__(self):

        #keep dict with known classes and their respective models
        self.class_list = {}
        self.T_nov = 0.25
        self.sig_scale = 3

        self.debug = True
        self.debug_list = {}

    def add_observation(self,obs):
        """obs - observation dict with fields
                name - class name as a string
                data - xy numpy array with observation"""

        if obs["name"] in self.class_list:
            self.class_list[obs["name"]].update(obs["data"])

            if self.debug:
                self.debug_list[obs["name"]].append(obs["data"])

        else:
            n = obs["data"].shape[0]
            self.class_list[obs["name"]] = igmm.IGMM(n, self.sig_scale, self.T_nov)
            self.class_list[obs["name"]].update(obs["data"])

            if self.debug:
                self.debug_list[obs["name"]] = [obs["data"]]


    def plot_layer(self,ax):
        colorlist = ['b','g','r','m','k','y','c']
        count = 0
        m = len(colorlist)
        for name,model in self.class_list.iteritems():
            color_idx = count % m
            count += 1
            model.plot(ax,color=colorlist[color_idx])

            if self.debug:
                X = np.asarray(self.debug_list[name])
                ax.scatter(X[:, 0], X[:, 1], s=20,c=colorlist[color_idx],label=name)
            else:
                pass
                #figure scaling doesn't work properly when only plotting the ellipses, idk why but it would be good to fix


#DATA_FILE = "../data/RegionLocations.csv"
DATA_FILE = "../data/ObjectLocations.csv"

if __name__ == "__main__":

    # load data from file
    data = GenerateData.load_csv(DATA_FILE)
    # add some noise
    c = 0.3 * np.identity(2)
    noisy_data = GenerateData.noisy_observations(data, 20, c, True)

    #initialize layer
    L = Layer()

    #add data points
    for pt in noisy_data:
        L.add_observation(pt)

    #plot
    ax = plt.subplot(111)
    L.plot_layer(ax)
    plt.show()
