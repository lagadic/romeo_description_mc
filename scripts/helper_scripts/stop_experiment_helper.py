#! /usr/bin/env python
from ask_user import ask_user

def stopMotion(robots, qpsolver, postureTask, comTaskSp, lastJointPositions):
  print 'Motion stopped, please put the robot in the air'
  hsCB = ask_user.askUserNonBlock('half_sitting', 'HS')

  qpsolver.solver.resetTasks()
  postureTask.posture(lastJointPositions)
  postureTask.stiffness(1.)

  # tasks for stopping and being statically stable
  if comTaskSp is not None:
    qpsolver.solver.addTask(comTaskSp)
  qpsolver.solver.addTask(postureTask)
  qpsolver.solver.updateTasksNrVars(robots.mbs)

  return hsCB

def goHalfSitting(qpsolver, postureTask, halfSittingJointPositions, \
           removeConstraintList, addConstraintList):
  print 'Going to half-sitting'
  postureTask.posture(halfSittingJointPositions)
  postureTask.stiffness(1.)
  for remConst in removeConstraintList:
    qpsolver.removeConstraintSet(remConst)
  for addConst in addConstraintList:
    qpsolver.addConstraintSet(addConst)
  qpsolver.setContacts([])
  qpsolver.update()
