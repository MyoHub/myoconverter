""" Contains the `MovingPathPoint` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from lxml import etree
import os

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import vec2str
from myoconverter.xml.path_points.utils import get_moving_path_point_dependency
from myoconverter.xml.utils import get_body
from myoconverter.xml.path_points.PathPoint import PathPoint
from myoconverter.xml import config as cfg


class MovingPathPoint(IParser):
  """ This class parses and converts the OpenSim `MovingPathPoint` XML element to MuJoCo. """

  def __init__(self):
    self._pathpoint = PathPoint()

  def parse(self, xml, tendon, force_name, dependencies=None, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `MovingPathPoint` XML element
    :param tendon: MuJoCo `tendon/spatial` XML element
    :param force_name: Name of the actuator this path point belongs to
    :param dependencies: A dict containing joint-wise dependency info (used by ConditionalPathPoints)
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Create an output dir for plots (if 1. this is an actual MovingPathPoint and not a ConditionalPathPoint
    # modelled as one, 2. MovingPathPoints are not treated as normal PathPoints, and 3. the output dir doesn't exist)
    output_dir = os.path.join(cfg.OUTPUT_PLOT_FOLDER, "moving_path_points")
    if dependencies is None and not cfg.TREAT_AS_NORMAL_PATH_POINT and not os.path.isdir(output_dir):
      os.makedirs(output_dir)

    # Get path point dependencies
    if dependencies is None:
      joint_x, polycoef_x, range_x = get_moving_path_point_dependency(xml, "x_location", "socket_x_coordinate",
                                                                      cfg, output_dir)
      joint_y, polycoef_y, range_y = get_moving_path_point_dependency(xml, "y_location", "socket_y_coordinate",
                                                                      cfg, output_dir)
      joint_z, polycoef_z, range_z = get_moving_path_point_dependency(xml, "z_location", "socket_z_coordinate",
                                                                      cfg, output_dir)
    else:
      joint_x, polycoef_x, range_x = dependencies["x"]
      joint_y, polycoef_y, range_y = dependencies["y"]
      joint_z, polycoef_z, range_z = dependencies["z"]

    # Check if we should treat moving path points as normal path points
    if cfg.TREAT_AS_NORMAL_PATH_POINT:
      logger.info(f"Treating MovingPathPoint {xml.attrib['name']}, as a normal PathPoint")
      self._pathpoint.parse(xml, tendon, force_name,
                            pos=np.array([np.mean(range_x), np.mean(range_y), np.mean(range_z)]))
      return

    # Get socket parent frame
    socket_parent_frame = xml.find("socket_parent_frame").text

    # Get the mujoco body socket_parent_frame references to
    body = get_body(cfg.OPENSIM, cfg.M_WORLDBODY, socket_parent_frame)

    # Create a new "imaginary" body for this moving site. Need to be careful with setting the body name, there can be
    # multiple MovingPathPoints with the same name in the same body. Use muscle name to create a unique name.
    new_body = etree.SubElement(body, "body", name=f"{force_name}_{xml.attrib['name']}")

    # Add an invisible geom with no collision properties; required so that mass/inertial properties are saved when xml
    # file is saved in later optimisation stages. Set contype=2 and conaffinity=2 so that these won't collide
    # with other geoms. Note! They can still collide with each other. But the position of the imaginary bodies are
    # controlled by equality constraints so shouldn't be a problem? Also, the bodies are likely to be far enough from
    # each other.
    etree.SubElement(new_body, "geom", size="0.0005 0.0005 0.0005", rgba="0.0 0.0 1.0 0.0", contype="2", conaffinity="2")

    # Add joints and respective joint constraints -- unless the range is very small (less than 1 mm), in which case just
    # modify the position of the new body
    pos = np.zeros(3)

    # x
    if self._small_range(range_x):
      pos[0] = range_x[0]
    else:
      self._add_joint(new_body, f"{force_name}_{xml.attrib['name']}_x", joint_x, polycoef_x, range_x, "1 0 0")

    # y
    if self._small_range(range_y):
      pos[1] = range_y[0]
    else:
      self._add_joint(new_body, f"{force_name}_{xml.attrib['name']}_y", joint_y, polycoef_y, range_y, "0 1 0")

    # z
    if self._small_range(range_z):
      pos[2] = range_z[0]
    else:
      self._add_joint(new_body, f"{force_name}_{xml.attrib['name']}_z", joint_z, polycoef_z, range_z, "0 0 1")

    # Set pos of new_body
    new_body.attrib["pos"] = vec2str(pos)

    # Add this path point to the body, and to the given tendon
    etree.SubElement(new_body, "site", name=f"{force_name}_{xml.attrib['name']}")
    etree.SubElement(tendon, "site", site=f"{force_name}_{xml.attrib['name']}")

  def _add_joint(self, new_body, name, joint, polycoef, range, axis):
    """ Add (imaginary) joint to MuJoCo XML file.

    :param new_body: MuJoCo `body` XML element
    :param name: Name of joint
    :param joint: Name of independent joint
    :param polycoef: Quartic function to describe how this joint moves wrt to `joint`
    :param range: Range of movement
    :param axis: Axis of joint
    :return: None
    """
    etree.SubElement(new_body, "joint",
                     name=name,
                     type="slide",
                     axis=axis,
                     range=vec2str(range))
    etree.SubElement(cfg.M_EQUALITY, "joint",
                     joint1=name,
                     joint2=joint,
                     polycoef=vec2str(polycoef),
                     solimp="0.9999 0.9999 0.001 0.5 2")

  def _small_range(self, vec):
    """ Check whether the range is very small.

    :param vec: Two element vector describing a range
    :return: Boolean indicating whether the range is very small -- less than one millimeter (< 0.001)
    """
    if np.abs(vec[1] - vec[0]) < 0.001:
      return True
    else:
      return False
