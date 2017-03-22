import dill as pickle
import SingleGaussian
import numpy as np

test = True
file_name = "temp_file.pkl"

if test:
    #some quick testing

    #set up layer
    cfg = {"type":"static", "model":"single"}
    layer = SingleGaussian.SimpleLayer(cfg)

    #add observations
    obs = {"name": "Alex","data": np.array([1, 2, 3])}
    layer.add_observation(obs)
    obs = {"name": "Alex", "data": np.array([5.0, 3.123, 0.1234])}
    layer.add_observation(obs)
    obs = {"name": "Alex", "data": np.array([5.234, 8.4323, 0.094723])}
    layer.add_observation(obs)
    obs = {"name": "Alex", "data": np.array([1.234, 2.4323, 6.094723])}
    layer.add_observation(obs)

    obs = {"name": "Joe", "data": np.array([3,8])}
    layer.add_observation(obs)
    obs = {"name": "Joe", "data": np.array([6.2,9.234])}
    layer.add_observation(obs)
    obs = {"name": "Joe", "data": np.array([8.213, 1.234])}
    layer.add_observation(obs)

    obs = {"name": "Fred", "data": np.array([3.0])}
    layer.add_observation(obs)
    obs = {"name": "Fred", "data": np.array([3.1])}
    layer.add_observation(obs)

    #print results
    print layer.get_params("Alex")
    print layer.get_params("Joe")
    print layer.get_params("Fred")

    #save to file
    with open(file_name,"wb") as f:
        tmp = pickle.dump(layer, f, protocol=pickle.HIGHEST_PROTOCOL)

else:

    #open file and print results
    with open(file_name, "rb") as f:
        layer = pickle.load(f)
        print layer.get_params("Alex")
        print layer.get_params("Joe")
        print layer.get_params("Fred")
