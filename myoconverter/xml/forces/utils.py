""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `ForceSet`

Muscle conversion analysis by Florian Fischer and Miroslav Bachinski.

@author: Aleksi Ikkala
"""

import numpy as np
from scipy.optimize import minimize


def mujoco_LO_loss(length_range, range, optimal_fiber_length, tendon_slack_length, pennation_angle):
  """
  Computes squared Euclidean distance between MuJoCo and OpenSim model,
  regarding both optimal fiber length and constant tendon length/tendon slack length.

  Original code for this function was provided by Florian Fischer (2022).

  :param length_range: array of MuJoCo tendon length (=complete actuator length) ranges
  :param range: Operating length of muscle
  :param optimal_fiber_length: OpenSim optimal fiber length
  :param tendon_slack_length: OpenSim tendon slack length (or any reasonable constant tendon lengths)
  :param pennation_angle: OpenSim pennation angle at optimum
          (i.e., angle between tendon and fibers at optimal fiber length expressed in radians)
  :param use_optPennationAngle: Boolean; if this set to True, MuJoCo optimal fiber lengths LO should match
          OpenSim optimal fiber lengths LO_osim * cos(OpenSim pennation angle at optimum); otherwise, LO should match LO_osim
  :return: squared (unweighted) Euclidean distance of optimal fiber length and constant tendon lengths between MuJoCo and OpenSim
  """
  LO = estimate_fiber_length(length_range, range)
  LT = estimate_tendon_slack_length(length_range, range)

  if np.isnan(pennation_angle):
    pennation_angle = 0

  return np.linalg.norm(LO - optimal_fiber_length * np.cos(pennation_angle)) ** 2 + np.linalg.norm(
    LT - tendon_slack_length) ** 2


def estimate_fiber_length(length_range, range):
  """ Code by Florian Fischer """
  return (length_range[0] - length_range[1]) / (range[0] - range[1])


def estimate_tendon_slack_length(length_range, range):
  """ Code by Florian Fischer """
  return length_range[0] - range[0] * estimate_fiber_length(length_range, range)


def calculate_length_range(range, optimal_fiber_length, tendon_slack_length, pennation_angle):
  """ Length range computations by Florian Fischer """

  # Estimate length range if optimal fiber length and tendon slack length are defined
  if np.all(np.isfinite([optimal_fiber_length, tendon_slack_length, pennation_angle])):

    # Estimate actuator length ranges by minimizing error between mujoco and opensim optimal fiber length
    length_range = np.array([0.5, 2]) * tendon_slack_length
    sol = minimize(mujoco_LO_loss, length_range,
                   args=(range, optimal_fiber_length, tendon_slack_length, pennation_angle))

    if sol.success:
      return sol.x

  return np.array([np.nan, np.nan])
