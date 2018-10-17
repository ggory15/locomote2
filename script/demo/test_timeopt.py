# -*- coding: utf-8 -*-
# Copyright 2018, LAAS-CNRS
# Author: Pierre Fernbach


#from walkBauzil_hrp2_interStatic import *
#from darpa_line_hrp2_interStatic import *
#from flatGround_hrp2_interpSTATIC import *
#from plateform_hrp2_interp_testTransition import *
#from stairs10_hrp2_interStatic import * 
#from slalom_bauzil_hrp2_interStatic import *
from  talos.test_flat import *

from config import *
import time
from check_path import *

### run hpp-timeopt here ###
if True :
    if RUN_GENERATE_TIMEOPT and RUN_TIMEOPT:
        tStart = time.time()    
        import generate_timeopt_problem_rbprm as tp
        filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
        tp.generate_timeopt_problem(filename, r, True)
        tGenerateTimeopt = time.time() - tStart
    else:
        tGenerateTimeopt=0.
print "### run hpp-timeopt Done. ###"

#print "trajectory length from planning : "+str(fullBody.getTimeAtState(endState)/SPEED)

print "___________________________________"
#print "trajectory length: "+str(cs_out.contact_phases[-1].time_trajectory[-1])
print "planning          : "+str(tPlanning)
print "Contact sequence  : "+str(tInterpolateConfigs)
print "Time-optimization : "+str(tGenerateTimeopt)
