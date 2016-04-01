'''
Interpolator for the gains
Implements linear interpolation for the current and desired gains
within the defined number of iterations
TODO: maybe another smoothing function is better
'''
class GainInterpolator(object):
  def __init__(self, ordered_list, numIters):
    self.numIters = numIters
    self.iter = 0
    self.tasks = map(lambda x:x[0], ordered_list)
    self.currentGains = map(lambda x:x.stiffness(), self.tasks)
    self.nextGains = map(lambda x:x[1], ordered_list)
    self.diffs = [y2-y1 for y2,y1 in zip(self.nextGains, self.currentGains)]

  def oneStep(self):
    if self.iter <= self.numIters:
      for task, diff, curr in zip(self.tasks, self.diffs, self.currentGains):
        # linear increment
        task.stiffness((diff*self.iter/self.numIters)+curr)
      self.iter += 1
      return False
    else:
      for task, nxtG in zip(self.tasks, self.nextGains):
        # make sure there is no numerical error at the end
        task.stiffness(nxtG)
      return True



# TODO: create a base interpolator class for easier refactoring
class WeightInterpolator(object):
  def __init__(self, ordered_list, numIters):
    self.numIters = numIters
    self.iter = 0
    self.tasks = map(lambda x:x[0], ordered_list)
    self.currentWeights = map(lambda x:x.weight(), self.tasks)
    self.nextWeights = map(lambda x:x[1], ordered_list)
    self.diffs = [y2-y1 for y2,y1 in zip(self.nextWeights, self.currentWeights)]

  def oneStep(self):
    if self.iter <= self.numIters:
      for task, diff, curr in zip(self.tasks, self.diffs, self.currentWeights):
        # linear increment
        task.weight((diff*self.iter/self.numIters)+curr)
      self.iter += 1
      return False
    else:
      for task, nxtW in zip(self.tasks, self.nextWeights):
        # make sure there is no numerical error at the end
        task.weight(nxtW)
      return True