import locomote2
from hyq_config import *

from pinocchio.robot_wrapper import RobotWrapper
import pinocchio as se3

from numpy.linalg import norm, pinv, inv

robot = RobotWrapper(urdf_model_path, mesh_dir, root_joint=se3.JointModelFreeFlyer())
model = robot.model
data = robot.data
com_init = robot.com(robot.q0)
com_final =  com_init + np.matrix([0.01, 0.0, 0]).transpose()
MASS = robot.data.mass[0]

robot.initDisplay(loadModel=True)
robot.display(q0)

# for generating Time Optimization Problem
nstep = 5;
tp = locomote2.TimeoptProblem(nstep)
tp.init_com = com_init
tp.final_com = com_final
tp.mass = MASS

print(com_init)
# Adding each contact sequence
phase = locomote2.TimeoptPhase()
phase.ee_id = locomote2.EndeffectorID.LF
phase.start_time = 0.0
phase.end_time = 1.0
fid = model.getFrameId("lf_foot");
oMi = robot.framePosition(q0, fid);
phase.pos = oMi.translation
phase.rot = oMi.rotation
tp.phases[0] = phase

phase = locomote2.TimeoptPhase()
phase.ee_id = locomote2.EndeffectorID.RF
phase.start_time = 0.0
phase.end_time = 4.0
fid = model.getFrameId("rf_foot");
oMi = robot.framePosition(q0, fid);
phase.pos = oMi.translation
phase.rot = oMi.rotation
tp.phases[1] = phase

phase = locomote2.TimeoptPhase()
phase.ee_id = locomote2.EndeffectorID.LH
phase.start_time = 0.0
phase.end_time = 4.0
fid = model.getFrameId("lh_foot");
oMi = robot.framePosition(q0, fid);
phase.pos = oMi.translation
phase.rot = oMi.rotation
tp.phases[2] = phase

phase = locomote2.TimeoptPhase()
phase.ee_id = locomote2.EndeffectorID.RH
phase.start_time = 0.0
phase.end_time = 4.0
fid = model.getFrameId("rh_foot");
oMi = robot.framePosition(q0, fid);
phase.pos = oMi.translation
phase.rot = oMi.rotation
tp.phases[3] = phase

phase = locomote2.TimeoptPhase()
phase.ee_id = locomote2.EndeffectorID.LF
phase.start_time = 3.0
phase.end_time = 4.0
fid = model.getFrameId("lf_foot");
oMi = robot.framePosition(q0, fid);
phase.pos = oMi.translation + np.matrix([0.01, 0.0, 0]).transpose()
phase.rot = oMi.rotation
tp.phases[4] = phase


# For logging system
print ""
print "Write .dat file for Timeopt : ", DAT_PATH + '/' + DAT_NAME + ".dat"
tp.generateDATFile(DAT_PATH + '/' + DAT_NAME + ".dat")

# For loading default configuration file
tp.setTimeoptSolver(CONFIG_PATH+'/'+'cfg_timeopt_demo01.yaml', CONFIG_PATH+'/'+'default_solver_setting.yaml')
print "Load Configuration File : ", CONFIG_PATH+'/'+'cfg_timeopt_demo01.yaml'
tp.solve()

for k in range(tp.get_trajsize()):
    print("time", tp.get_time(k))
    print("linear momentum", tp.get_LMOM(k).transpose())
    print("angular momentum", tp.get_AMOM(k).transpose())
    print("COM", tp.get_COM(k).transpose())
    #print("estimated contact force ratio of lfoot", (tp.get_ContactForce(1, k).transpose()) )
    #0: rfoot, 1:lfoot, 2:rhand, 3:lhand

print("COM_final", com_final.transpose())