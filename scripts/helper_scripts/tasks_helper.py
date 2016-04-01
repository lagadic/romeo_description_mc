#! /usr/bin/env python
import spacevecalg as sva
import tasks
from eigen3 import Vector3d

# Helper function for using only a selection of joints in the task
def jointsSelector(robots, index, hl, jointsName):
  r = robots.robots[index]
  jointsId = map(r.jointIdByName, jointsName)
  return tasks.qp.JointsSelector(robots.mbs, index, hl, jointsId)

# Helper function for creating a position set point task
def positionTask(robots, robot_index, bodyName, pos, stiff, w, ef=Vector3d.Zero(), jointsName=[]):
  posTask = tasks.qp.PositionTask(robots.mbs, robot_index,
                                  robots.robots[robot_index].bodyIdByName(bodyName),
                                  pos, ef)
  if len(jointsName) > 0:
    posTaskSel = jointsSelector(robots, robot_index, posTask, jointsName)
    posTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, posTaskSel, stiff, w)
    return posTask, posTaskSp, posTaskSel
  posTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, posTask, stiff, w)
  return posTask, posTaskSp

# Helper function for creating a position trajectory tracking task
def positionTrajectoryTask(robots, robot_index, bodyName, pos, gainPos, gainVel, w, ef=Vector3d.Zero(), jointsName=[]):
  posTask = tasks.qp.PositionTask(robots.mbs, robot_index,
                                  robots.robots[robot_index].bodyIdByName(bodyName),
                                  pos, ef)
  if len(jointsName) > 0:
    posTaskSel = jointsSelector(robots, robot_index, posTask, jointsName)
    posTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, posTaskSel, gainPos, gainVel, w)
    return posTask, posTaskTr, posTaskSel
  posTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, posTask, gainPos, gainVel, w)
  return posTask, posTaskTr

# Helper function for creating orientation task
def orientationTask(robots, robot_index, bodyName, ori, stiff, w, jointsName=[]):
  oriTask = tasks.qp.OrientationTask(robots.mbs, robot_index,
                                     robots.robots[robot_index].bodyIdByName(bodyName),
                                     ori)
  if len(jointsName) > 0:
    oriTaskSel = jointsSelector(robots, robot_index, oriTask, jointsName)
    oriTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, oriTaskSel, stiff, w)
    return oriTask, oriTaskSp, oriTaskSel

  oriTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, oriTask, stiff, w)
  return oriTask, oriTaskSp

# Helper function for creating an orientation trajectory tracking task
def orientationTrajectoryTask(robots, robot_index, bodyName, ori, gainPos, gainVel, w):
  oriTask = tasks.qp.OrientationTask(robots.mbs, robot_index,
                                     robots.robots[robot_index].bodyIdByName(bodyName),
                                     ori)
  oriTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, oriTask, gainPos, gainVel, w)
  return oriTask, oriTaskTr

# Helper function for creating a Center of Mass task
def comTask(robots, index, com, stiff, w):
  comTask = tasks.qp.CoMTask(robots.mbs, index, com)
  comTaskSp = tasks.qp.SetPointTask(robots.mbs, index, comTask, stiff, w)
  return comTask, comTaskSp

# Helper function for creating a Center of Mass tracking task
def comTrajectoryTask(robots, robot_index, com, gainPos, gainVel, w):
  comTask = tasks.qp.CoMTask(robots.mbs, robot_index, com)
  comTaskTr = tasks.qp.TrajectoryTask(robots.mbs, robot_index, comTask,
                                      gainPos, gainVel, w)
  return comTask, comTaskTr

#TODO: try a trajectory task version also
# Helper function for creating a Position-Based Visual Servoing task
def pbvsTask(robots, robot_index, bodyName, X_t_s, stiff, w, X_b_s=sva.PTransformd.Identity(), jointsName=[]):
  pbvsTask = tasks.qp.PositionBasedVisServoTask(robots.mbs, robot_index,
                                               robots.robots[robot_index].bodyIdByName(bodyName),
                                               X_t_s, X_b_s)
  if len(jointsName) > 0:
    pbvsTaskSel = jointsSelector(robots, robot_index, pbvsTask, jointsName)
    pbvsTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, pbvsTaskSel, stiff, w)
    return pbvsTask, pbvsTaskSp, pbvsTaskSel
  pbvsTaskSp = tasks.qp.SetPointTask(robots.mbs, robot_index, pbvsTask, stiff, w)
  return pbvsTask, pbvsTaskSp