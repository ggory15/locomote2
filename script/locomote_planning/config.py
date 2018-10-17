#from configs.walk_bauzil_stairs import *
#from configs.platform_config import *
#from configs.platform_hand_config import *
#from configs.darpa import *
from configs.timeopt_config import *
#from configs.random_test import *
#from configs.straight_walk_dynamic_planning_config import *
#from configs.stairs_config import *
#from configs.stairs10_bauzil_stairs import * 

#from configs.talos_flatGround import * 

# options for generate_contact_sequence :
SPEED=1.
DURATION_n_CONTACTS = 0.2 # percentage of time allocated to the movement of the com without moving the contacts
DISPLAY_CONTACTS = True
FORCE_STRAIGHT_LINE = False # DEBUG ONLY should be false

# options for generate_muscod_problem :
DISPLAY = True
DISPLAY_CURVES = True
DISPLAY_CONES = False
VERBOSE = 0
STOP_EACH_PHASE=False
LOAD_PREVIOUS_SEQ = True
MUSCOD_KINODYNAMIC = False


# options for comrrt :

RUN_GENERATE_MUSCOD=True
RUN_GENERATE_TIMEOPT=True

RUN_MUSCOD = False
RUN_TIMEOPT = True

DISPLAY_MUSCOD = True
DISPLAY_TRAJ=True
DISPLAY_MOTION_HPP = False
DISPLAY_FEET_TRAJ_INIT_GUESS=False
DISPLAY_FEET_TRAJ=True

# options for whole body :
CONSTRAINT_ROOT_YAW = False
WB_DISPLAY = True
WB_DISPLAY_PLOT = True
WB_DISPLAY_MOTION_AS_COMPUTED = False
WB_DISPLAY_TRAJ=True
WB_PG=True
WB_EXPORT_OPENHRP=True
WB_DISPLAY_STEPPING_STONE = True
WB_DISPLAY_PLANNING_ENV = True
WB_VERBOSE=False
ALPHA_ANGULAR_MOMENTUM = 0.1
