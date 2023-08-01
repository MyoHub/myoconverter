""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `PathPointSet`

@author: Aleksi Ikkala
"""

import numpy as np
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
import matplotlib
import matplotlib.pyplot as pp
import os

from loguru import logger

from myoconverter.xml.utils import str2vec, fit_spline

matplotlib.use("agg")
pp.ioff()


def update_moving_path_point_location(coordinate_name, path_point):
  """ Calculate median for a coordinate of a MovingPathPoint

  :param coordinate_name: Coordinate name (related x, y, or z coordinate)
  :param path_point: OpenSim `MovingPathPoint` XML element
  :return: Median of given coordinate
  :raises: RuntimeError: If a Coordinate is not found
  :raises: NotImplementedError: If a path point is not formatted as expected
  :raises: NotImplementedError: If a function type is other than 'SimmSpline', 'NaturalCubicSpline', or
  'PiecewiseLinearFunction'

  """

  # Make sure the coordinate exists
  if path_point.find(coordinate_name) is None:
    logger.critical(f"Coordinate f{coordinate_name} was not found in pathpoint {path_point.attrib['name']}")
    raise RuntimeError

  # Check if the function is wrapped in a multiplier function
  mult_fn = path_point.find(f"{coordinate_name}/MultiplierFunction")
  multiplier = 1
  if mult_fn is not None:
    multiplier = float(mult_fn.find("scale").text)
    if multiplier != 1:
      logger.warning(f"Encountered a MultiplierFunction with scale != 1 in pathpoint {path_point.attrib['name']}."
             f"This implementation has not been tested/validated, you better check it out yourself")

  # Parse x and y values
  if path_point.find(".//x") is None:
    logger.critical(f"Path point {path_point.attrib['name']} is not formatted as expected")
    raise NotImplementedError

  x_values = str2vec(path_point.find(f".//{coordinate_name}/*/x").text)
  y_values = str2vec(path_point.find(f".//{coordinate_name}/*/y").text) * multiplier

  # Figure out function type
  function_type = path_point.find(".//x").getparent().tag

  if function_type not in {"SimmSpline", "NaturalCubicSpline", "PiecewiseLinearFunction"}:
    logger.critical(f"Function type {function_type} not implemented (in pathpoint {path_point.attrib['name']})")
    raise NotImplementedError

  # Fit a cubic spline (if more than 2 values and function_type is spline), otherwise fit a piecewise linear line
  if len(x_values) > 3 and function_type in {"SimmSpline", "NaturalCubicSpline"}:
    mdl = UnivariateSpline(x_values, y_values)
  else:
    mdl = interp1d(x_values, y_values, kind="linear", fill_value="extrapolate")

  # Return the median of fit inside given range
  x = np.linspace(x_values[0], x_values[-1], 1000)
  return np.median(mdl(x))

def get_moving_path_point_dependency(path_point, coordinate_name, socket_name, cfg, output_dir):
  """ Get a MovingPathPoint's dependency on an independent coordinate.

  :param path_point:  OpenSim `MovingPathPoint` XML element
  :param coordinate_name: Name coordinate ("x_location", "y_location", or "z_location")
  :param socket_name: Name of the specific coordinate socket ("socket_x_coordinate", "socket_y_coordinate", or
  "socket_z_coordinate")
  :param cfg: Contains all the global config definitions
  :param output_dir: Output directory for plots
  :return: Name of the independent coordinate (joint) `path_point` is dependent on, quartic polynomial of the
  dependency, and movement range
  :raises: RuntimeError: If a Coordinate was not found
  :raises: NotImplementedError: If a path point is not formatted as expected
  :raises: RuntimeError: If a MuJoCo joint that corresponds to a Coordinate cannot be found
  """

  # Make sure the coordinate exists
  if path_point.find(coordinate_name) is None:
    logger.critical(f"Coordinate f{coordinate_name} was not found in path point {path_point.attrib['name']}")
    raise RuntimeError

  # Check if the function is wrapped in a multiplier function
  mult_fn = path_point.find(f"{coordinate_name}/MultiplierFunction")
  multiplier = 1
  if mult_fn is not None:
    multiplier = float(mult_fn.find("scale").text)
    if multiplier != 1:
      logger.warning(f"Encountered a MultiplierFunction with scale != 1 in path point {path_point.attrib['name']}."
             f"This implementation has not been tested/validated, you better check it out yourself")

  # Parse x and y values
  if path_point.find(".//x") is None:
    logger.critical(f"Path point {path_point.attrib['name']} is not formatted as expected")
    raise NotImplementedError

  x_values = str2vec(path_point.find(f".//{coordinate_name}//x").text)
  y_values = str2vec(path_point.find(f".//{coordinate_name}//y").text) * multiplier

  # Figure out function type
  function_type = path_point.find(".//x").getparent().tag

  if function_type not in {"SimmSpline", "NaturalCubicSpline", "PiecewiseLinearFunction"}:
    logger.critical(f"Function type {function_type} not implemented (in {path_point.tag} {path_point.attrib['name']}). "
                    f"Behaviour of this type of function has not been tested.")

  # Get polycoef
  fit, polycoef, range = fit_spline(x_values, y_values)

  # Find the independent coordinate
  independent_coordinate = path_point.find(socket_name).text.split("/")[-1]

  # Make sure the coordinate has been designated as one of the MuJoCo model joints
  if cfg.M_WORLDBODY.find(f".//joint[@name='{independent_coordinate}']") is None:
    logger.critical(f"Could not find a MuJoCo joint that corresponds to independent coordinate {independent_coordinate}")
    raise RuntimeError

  # Plot figures of the approximations (if necessary)
  if not cfg.TREAT_AS_NORMAL_PATH_POINT:
    fig = plot_figure(x_values, y_values, fit, independent_coordinate, path_point.attrib["name"], coordinate_name[0])
    pp.savefig(os.path.join(output_dir, f"{path_point.attrib['name']}_{coordinate_name[0]}.svg"))
    pp.close(fig)

  return independent_coordinate, polycoef, range


def plot_figure(x_values, y_values, fit, independent_joint, path_point_name, coord):

  # Initialise figure
  fig = pp.figure(figsize=(10, 8))

  # Calculate the quartic approximation
  x_approx = np.linspace(min(x_values), max(x_values), 100)
  y_approx = fit(x_approx)

  # Plot the OpenSim data points and approximation
  pp.plot(x_values, y_values, '.', markersize=10, label=f"OpenSim movement function data points")
  pp.plot(x_approx, y_approx, label=f"Approximation of the movement function")
  pp.legend()
  pp.xlabel(f"Independent joint value ({independent_joint})")
  pp.ylabel(f"Dependent joint value ({path_point_name}_{coord})")
  pp.title(f"Approximation of the movement function of MovingPathPoint {path_point_name}, {coord} coordinate")

  return fig
