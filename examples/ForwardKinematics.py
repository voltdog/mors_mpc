import numpy as np
import os
import yaml

# read config
config = {}
current_working_directory = os.path.dirname(__file__)
config_path = current_working_directory+'/../config/robot_config.yaml'
try:
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
except:
    raise RuntimeError("Cannot find robot_config.yaml")
bx = config["bx"]
by = config["by"]
l1 = config["l1"]
l2 = config["l2"]
l3 = config["l3"]
d1 = config["d1"]
d2 = config["d2"]
d3 = config["d3"]

def fkine_R1(theta):
    q1 = theta[0]
    q2 = theta[1]
    q3 = theta[2]

    px = l1 + l2*np.sin(q2) + l3*np.sin(q2 + q3)
    py = -d1*np.cos(q1) - d2*np.cos(q1) - d3*np.cos(q1) + l2*np.sin(q1)*np.cos(q2) + l3*np.sin(q1)*np.cos(q2 + q3)
    pz = -d1*np.sin(q1) - d2*np.sin(q1) - d3*np.sin(q1) - l2*np.cos(q1)*np.cos(q2) - l3*np.cos(q2 + q3)*np.cos(q1)

    return np.array([px+bx, py-by, pz])

def fkine_L1(theta):
    q1 = theta[3]
    q2 = theta[4]
    q3 = theta[5]

    px = l1 - l2*np.sin(q2) - l3*np.sin(q2 + q3)
    py = d1*np.cos(q1) + d2*np.cos(q1) + d3*np.cos(q1) + l2*np.sin(q1)*np.cos(q2) + l3*np.sin(q1)*np.cos(q2 + q3)
    pz = d1*np.sin(q1) + d2*np.sin(q1) + d3*np.sin(q1) - l2*np.cos(q1)*np.cos(q2) - l3*np.cos(q2 + q3)*np.cos(q1)

    return np.array([px+bx, py+by, pz])

def fkine_R2(theta):
    q1 = theta[6]
    q2 = theta[7]
    q3 = theta[8]

    px = -l1 + l2*np.sin(q2) + l3*np.sin(q2 + q3)
    py = -d1*np.cos(q1) - d2*np.cos(q1) - d3*np.cos(q1) - l2*np.sin(q1)*np.cos(q2) - l3*np.sin(q1)*np.cos(q2 + q3)
    pz = d1*np.sin(q1) + d2*np.sin(q1) + d3*np.sin(q1) - l2*np.cos(q1)*np.cos(q2) - l3*np.cos(q2 + q3)*np.cos(q1)

    return np.array([px-bx, py-by, pz])

def fkine_L2(theta):
    q1 = theta[9]
    q2 = theta[10]
    q3 = theta[11]

    px = -l1 - l2*np.sin(q2) - l3*np.sin(q2 + q3)
    py = d1*np.cos(q1) + d2*np.cos(q1) + d3*np.cos(q1) - l2*np.sin(q1)*np.cos(q2) - l3*np.sin(q1)*np.cos(q2 + q3)
    pz = -d1*np.sin(q1) - d2*np.sin(q1) - d3*np.sin(q1) - l2*np.cos(q1)*np.cos(q2) - l3*np.cos(q2 + q3)*np.cos(q1)

    return np.array([px-bx, py+by, pz])