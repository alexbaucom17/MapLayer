import dill as pickle
from MapLayerTmp import GaussianModel
import numpy as np

test = False
file_name = "temp_file.pkl"

if test:
    #some quick testing
    mu = np.array([0,0,0])
    sig = 0.5 * np.identity(3)
    GM = GaussianModel(mu,sig)
    #print GM.get_mean()
    #print GM.get_cov()
    GM.update(np.array([1, 2, 3]))
    GM.update(np.array([2, 2, 2]))
    GM.update(np.array([3, 2, 1]))
    GM.update(np.array([100, 2, -8.4]))
    print GM.get_mean()
    print GM.get_cov()

    with open(file_name,"wb") as f:
        tmp = pickle.dump(GM, f, protocol=pickle.HIGHEST_PROTOCOL)

else:

    with open(file_name, "rb") as f:
        GM2 = pickle.load(f)
        print GM2.get_mean()
        print GM2.get_cov()