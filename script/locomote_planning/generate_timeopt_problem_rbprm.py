# -*- coding: utf-8 -*-
# Copyright 2018, LAAS-CNRS
# Author: Justin Carpentier, Pierre Fernbach, Sanghyun Kim

from config import *

import numpy as np
from numpy.linalg import norm, pinv, inv
import math

import pinocchio as se3
from pinocchio import SE3
from pinocchio.utils import *

if robotName == "hrp2":
  from robot_config.hrp2014_wrapper import Hrp2014Wrapper as Wrapper
elif robotName == "talos":
  from robot_config.pyrene_wrapper import PyreneWrapper as Wrapper
else:
  raise ValueError('Robot name doesnt define a known robot model.')

import os, time
import locomote
import locomote2

from locomote import WrenchCone,SOC6,ControlType,IntegratorType,ContactPatch, ContactPhaseHumanoid, ContactSequenceHumanoid

if DISPLAY_CURVES:
  import matplotlib.pyplot as plt
  from mpl_toolkits.mplot3d import Axes3D
  from gui_tools import *
  from plot_tools import *


def DrawCOMSphere(Viewer):
  Viewer.client.gui.addSphere("0_scene_hpp_/COM", 0.03, Viewer.color.green)
  Viewer.client.gui.applyConfiguration("0_scene_hpp_/COM", [0, 0, 0, 1, 0, 0, 0])
  Viewer.client.gui.refresh()


def UpdateCOMDisplay(pos, Viewer):
  Viewer.client.gui.applyConfiguration("0_scene_hpp_/COM", [pos[0], pos[1], pos[2], 1, 0, 0, 0])
  Viewer.client.gui.refresh()

def fileExists(filename):
  import os.path
  return os.path.isfile(filename)

class Empty:
  None

import sys, termios, tty, os, time

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

robot = Wrapper(urdf_model_path, mesh_dir)

# alias
model = robot.model
data = robot.data
robot.com(robot.q0)
MASS = robot.data.mass[0]

q = robot.q0.copy()
q_init = q

def generate_timeopt_problem(filename, r, isInit=True,isFinal=True):
  print "load previous contact sequence : ",filename
  cs = ContactSequenceHumanoid(0)
  cs.loadFromXML(filename,CONTACT_SEQUENCE_XML_TAG)

  # set robot.q0 COM position to the initial state
  q_init[0:3] = cs.contact_phases[0].reference_configurations[0][0:3]
  n_steps = cs.size()

  com0 = cs.contact_phases[0].init_state[:3]
  q_ref = cs.contact_phases[0].reference_configurations[0]

  posture_l = []
  posture_l.append(q_ref)

  # this step must have the load YAML and generate TimeOptProblem ~(-_-~)
  t0 = 0.
  num_phases = 0;
  RF_time = [];
  LF_time = [];

  time_lf = 0.0;
  time_rf = 0.0;
  duration_ds_r = 0.0;
  duration_ds_l = 0.0;

  duration_ss_r = 0.0;
  duration_ss_l = 0.0;

  for k in range(n_steps):
    contact_phase = cs.contact_phases[k]
    if k < n_steps - 1:
      next_phase = cs.contact_phases[k + 1]
    num_active_phases = contact_phase.numActivePatches()

    assert (num_active_phases > 0)
    phase_type = locomote.HumanoidPhaseType.HUMANOID_PHASE_UNDEFINED

    # Determine the phase type
    if num_active_phases == 1:
      phase_type = locomote.HumanoidPhaseType.SINGLE_SUPPORT
    elif num_active_phases == 2:
      phase_type = locomote.HumanoidPhaseType.DOUBLE_SUPPORT
    elif num_active_phases == 3:
      phase_type = locomote.HumanoidPhaseType.TRIPLE_SUPPORT
    elif num_active_phases == 4:
      phase_type = locomote.HumanoidPhaseType.QUADRUPLE_SUPPORT
    else:
      assert False, "Must never happened"

    if phase_type == locomote.HumanoidPhaseType.SINGLE_SUPPORT:
      assert k > 0, "This phase must no be the initial one"
      assert k < n_steps - 1, "This phase must no be the final one"

      if contact_phase.RF_patch.active:
        duration_ss_r = max(contact_phase.time_trajectory[0], DURATION_SS)
        time_lf = time_lf + duration_ss_r
      elif contact_phase.LF_patch.active:
        duration_ss_l = max(contact_phase.time_trajectory[0], DURATION_SS)
        time_rf = time_rf + duration_ss_l

    elif phase_type == locomote.HumanoidPhaseType.DOUBLE_SUPPORT:
      if not next_phase.RF_patch.active:
        duration_ds_l = max(contact_phase.time_trajectory[0], DURATION_DS)
      elif not next_phase.LF_patch.active:
        duration_ds_r = max(contact_phase.time_trajectory[0], DURATION_DS)
      if k == 0 and isInit:
        if not next_phase.RF_patch.active:
          duration_ds_l = DURATION_INIT
        elif not next_phase.RF_patch.active:
          duration_ds_r = DURATION_INIT

      if k == n_steps - 1 and isFinal:  # last phase
        duration_ds_fin = DURATION_FINAL

      if not next_phase.RF_patch.active:
        RF_time.append(np.matrix([time_rf, time_rf + duration_ds_l + duration_ss_r + duration_ds_r]))
        time_rf = time_rf + duration_ds_l + duration_ss_r + duration_ds_r
      elif not next_phase.LF_patch.active:
        LF_time.append(np.matrix([time_lf, time_lf + duration_ds_l + duration_ss_l + duration_ds_r]))
        time_lf = time_lf + duration_ds_l + duration_ss_l + duration_ds_r

      if k == n_steps -1 :
        if cs.contact_phases[k-1].RF_patch.active:
          RF_time.append(np.matrix([time_rf, time_rf + duration_ds_fin + duration_ss_r + duration_ds_r]))
          LF_time.append(np.matrix([time_lf, time_lf + duration_ds_fin]))
        elif cs.contact_phases[k-1].LF_patch.active:
          LF_time.append(np.matrix([time_lf, time_lf + duration_ds_fin + duration_ss_l + duration_ds_l]))
          RF_time.append(np.matrix([time_rf, time_rf + duration_ds_fin]))

    elif phase_type == locomote.HumanoidPhaseType.TRIPLE_SUPPORT:
      duration = max(contact_phase.time_trajectory[0], DURATION_TS)
    else:
      assert False, "Must never happened"

  RF = [];
  LF = [];
  for k in range((n_steps - 1) /2):
    if not cs.contact_phases[2*k + 1].RF_patch.active:
      RF.append(cs.contact_phases[2 * k].RF_patch.placement)

    if not cs.contact_phases[2 * k + 1].LF_patch.active:
      LF.append(cs.contact_phases[2 * k].LF_patch.placement)

  RF.append(cs.contact_phases[n_steps-1].RF_patch.placement)
  LF.append(cs.contact_phases[n_steps-1].LF_patch.placement)

  tp = locomote2.TimeoptProblem(len(RF) + len(LF))

  tp.init_com = cs.contact_phases[0].init_state[0:3]
  tp.final_com = cs.contact_phases[n_steps-1].final_state[0:3]
  tp.mass = MASS
  for i in range(len(RF)):
    phase = locomote2.TimeoptPhase()
    phase.ee_id = locomote2.EndeffectorID.RF
    phase.start_time = RF_time[i][0, 0]
    phase.end_time = RF_time[i][0, 1]
    phase.pos = RF[i].translation
    phase.rot = RF[i].rotation
    tp.phases[i] = phase

  for i in range(len(LF)):
    phase = locomote2.TimeoptPhase()
    phase.ee_id = locomote2.EndeffectorID.LF
    phase.start_time = LF_time[i][0, 0]
    phase.end_time = LF_time[i][0, 1]
    phase.pos = LF[i].translation
    phase.rot = LF[i].rotation
    tp.phases[i+len(LF)] = phase

  print ""
  print "Write .dat file for Timeopt : ", DAT_PATH + '/' + DAT_NAME + ".dat"
  tp.generateDATFile(DAT_PATH + '/' + DAT_NAME + ".dat")

  tp.setTimeoptSolver(DAT_PATH+'/'+'cfg_timeopt_demo01.yaml', DAT_PATH+'/'+'default_solver_setting.yaml')
  print "Load Configuration File : ", DAT_PATH+'/'+'cfg_timeopt_demo01.yaml'
  tp.solve()

  state=[];
  s_time=0.0;
  cnt = 0;
  for k in range(tp.get_trajsize()):
      if norm(tp.get_ContactForce(0, k)) > 0.0 and norm(tp.get_ContactForce(1, k)) > 0.0:
        state.append('ds')
      elif  norm(tp.get_ContactForce(0, k)) > 0.0 and norm(tp.get_ContactForce(1, k)) == 0.0:
        state.append('ssr')
      elif norm(tp.get_ContactForce(1, k)) > 0.0 and norm(tp.get_ContactForce(0, k)) == 0.0:
        state.append('ssl')

  u = [0] * 6
  x = [0] * 9
  for k in range(tp.get_trajsize()):
      if k == 0 :
          cs.contact_phases[cnt].time_trajectory[0] = tp.get_time(k)
      else:
          if not state[k-1] == state[k]:
              cnt = cnt+1
              cs.contact_phases[cnt].time_trajectory[0] = (tp.get_time(k))
          else:
              cs.contact_phases[cnt].time_trajectory.append(tp.get_time(k))

      u[0:3] = tp.get_LMOM(k).tolist()
      u[3:6] = tp.get_AMOM(k).tolist()
      cs.contact_phases[cnt].control_trajectory.append(np.matrix(u))

      x[0:3] = tp.get_COM(k).tolist()
      x[3:6] = tp.get_LMOM(k).tolist()
      if k == 0:
        x[6:9] = ((tp.get_LMOM(k)/MASS) / tp.get_time(k)).tolist()
      x[6:9] = (((tp.get_LMOM(k)/MASS) - (tp.get_LMOM(k-1)/MASS)) / (tp.get_time(k)-tp.get_time(k-1))).tolist()
      cs.contact_phases[cnt].state_trajectory.append(np.matrix(x))

  filename = OUTPUT_DIR + "/" + '1' + OUTPUT_SEQUENCE_FILE
  cs.saveAsXML(filename, "ContactSequence")
  print("")

  if DISPLAY:
    traj_node = "world/trajs"
    r.client.gui.createGroup(traj_node)
    pos = []
    for k in range(0, cs.size()):
      phase = cs.contact_phases[k]
      pos = pos + [phase.init_state[0:3].transpose().tolist()[0]]
      pos = pos + [phase.final_state[0:3].transpose().tolist()[0]]

    nameCurve1 = traj_node + "/init_COM"
    r.client.gui.addCurve(nameCurve1, pos, [0.1, 0.5, 0.1, 1.0])
    r.client.gui.setVisibility(nameCurve1, "ALWAYS_ON_TOP")
    r.client.gui.addToGroup(nameCurve1, r.sceneName)
    r.client.gui.refresh()
'''
    pos = []
    for k in range(0, tp.get_trajsize()-1):
      pos = pos + [tp.get_COM(k)[0:3].transpose().tolist()[0]]
      pos = pos + [tp.get_COM(k+1)[0:3].transpose().tolist()[0]]

    nameCurve = traj_node + "/calculate_COM"
    r.client.gui.addCurve(nameCurve, pos, [1, 0.0, 0.0, 1.0])
    r.client.gui.setVisibility(nameCurve, "ALWAYS_ON_TOP")
    r.client.gui.addToGroup(nameCurve, r.sceneName)
    r.client.gui.refresh()
    DrawCOMSphere(r)
    for k in range(0, tp.get_trajsize()-1):
      UpdateCOMDisplay(tp.get_COM(k)[0:3].transpose().tolist()[0], r)
      time.sleep(tp.get_time(k+1)-tp.get_time(k))  # for draw
'''






