class Saturation(object):
  '''
  This saturates the inputVector according to the defined limits, meaning that
  the values are rescaled according to the maximum limit violation.
  '''
  def __init__(self, limits):
    self.limits = limits

  def __call__(self, inputVector):
    # find maximum limit violation
    scale_factor = 1.
    for vec, lim in zip(inputVector, self.limits):
      if vec > lim:
        if lim != 0.: # interpret 0 limits as skip this
          ratio = abs(vec/lim)
          if ratio > scale_factor:
            scale_factor = ratio

    # rescale the input
    for i, v in enumerate(inputVector):
      inputVector[i] = inputVector[i]/scale_factor

    return inputVector