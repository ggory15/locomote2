from math import sqrt
import numpy as np
import os

model_path                  = [os.getcwd()+"/data"];
urdf_model_path                = model_path[0] + "/hyq_description/urdf/hyq.urdf";
mesh_dir = model_path

freeFlyer                   = True;
q0                          = np.matrix([[0.0 ,  0.0,  0.63,  0.  , -0.  ,  0.  ,  1.  , -0.51,  0.74,
                                          -0.93, -0.18, -0.38,  1.35, -0.36,  0.89, -1.19, -0.3 ,  0.19,
                                          0.79]]).T;

DAT_PATH = "/home/ggory15/DATA"
DAT_NAME = "test_hyq_sanghyun"
CONFIG_PATH = "/home/ggory15/HPP/src/locomote2/script/config"