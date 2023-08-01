""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `JointSet`

@author: Aleksi Ikkala
"""

from lxml import etree
from scipy.spatial.transform import Rotation
import matplotlib
import matplotlib.pyplot as pp
import numpy as np
import os

from myoconverter.xml.utils import str2vec, str2bool

matplotlib.use("agg")
pp.ioff()


def parse_coordinates(objects):
  """ Parse OpenSim `Coordinates`.

  :param objects: A list of OpenSim `Coordinate` XML elements
  :return: A dictionary of parsed `Coordinates`
  """
  # Return if there are no coordinates
  if objects is None:
    return dict()

  # Parse all coordinates defined for this joint. Default value is stored as "user" key. Default value is also used for
  # "_transform_value", which is used when a joint is locked
  coordinates = dict()
  for coordinate in objects:
    default_value = coordinate.find("default_value")
    coordinates[coordinate.attrib["name"]] = {
      "name": coordinate.attrib["name"],
      "range": str2vec(coordinate.find("range").text),
      "limited": str2bool(coordinate.find("clamped").text),
      "_locked": str2bool(coordinate.find("locked").text)}
    if default_value is not None:
      coordinates[coordinate.attrib["name"]].update({"user": float(default_value.text),
                                                     "_transform_value": float(default_value.text)})
  return coordinates

def lock_joint(params, M_EQUALITY):
  """ Lock a joint by adding an equality constraint.

  :param params: Joint parameters
  :param M_EQUALITY: Pointer to equality constraints in MuJoCo XML file
  :return: None
  """

  # A dependent joint/coordinate can be locked (since it's possible to toggle lock on/off), so we have to disable
  # any existing constraints of this particular joint
  constraint = M_EQUALITY.find(f".//*[@joint1='{params['name']}']")
  if constraint is not None:
    # Disable the other constraint
    constraint.attrib["active"] = "false"

  # Create the new constraint
  etree.SubElement(M_EQUALITY, "joint",
                   name=params["name"] + "_locked",
                   active="true",
                   joint1=params["name"],
                   polycoef=f"{params['_transform_value']} 0 0 0 0",
                   solimp="0.9999 0.9999 0.001 0.5 2")

def estimate_axis(socket_child_frame, axis):
  """ Estimate axis of MuJoCo joint.

  :param socket_child_frame: Child frame socket
  :param axis: Axis of the OpenSim joint element
  :return: MuJoCo joint axis
  """

  # Get orientation of child frame
  child_orientation = str2vec(socket_child_frame.find("orientation").text)

  # Turn into a rotation
  child_rotation = Rotation.from_euler("XYZ", child_orientation)

  # Return new axis
  return child_rotation.apply(axis)

def plot_and_save_figure(x_values, y_values, fit, params, independent_coordinate, output_dir):

  # Initialise figure
  fig = pp.figure(figsize=(10, 8))

  # Calculate the quartic approximation
  x_approx = np.linspace(min(x_values), max(x_values), 100)
  y_approx = fit(x_approx)

  # Plot original data points and approximation
  pp.plot(x_values, y_values, '.', markersize=10, label=f"OpenSim spline data points")
  pp.plot(x_approx, y_approx, label=f"Approximation of the spline")
  pp.legend()
  pp.xlabel(f"Independent joint value ({independent_coordinate})")
  pp.ylabel(f"Dependent joint value ({params['name']})")
  pp.title(f"Approximation of dependent joint {params['name']} movement wrt independent joint {independent_coordinate}")

  # Save figure
  pp.savefig(os.path.join(output_dir, f"{params['name']}.svg"))

  # Close figure
  pp.close(fig)

  return fig
