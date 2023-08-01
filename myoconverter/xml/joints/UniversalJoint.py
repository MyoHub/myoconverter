""" Contains the `UniversalJoint` parser.

@author: Aleksi Ikkala
"""

from copy import deepcopy
import numpy as np
from lxml import etree

from loguru import logger

from myoconverter.xml.joints.Joint import Joint
from myoconverter.xml.joints.utils import parse_coordinates, estimate_axis
from myoconverter.xml.utils import val2str, filter_keys


class UniversalJoint(Joint):
  """ This class parses and converts the OpenSim `UniversalJoint` XML element to MuJoCo. """

  def _parse(self, xml, socket_parent_frame, socket_child_frame, pointer):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `UniversalJoint` XML element
    :param socket_parent_frame: Parent frame socket
    :param socket_child_frame: Child frame socket
    :param pointer: A pointer to the MuJoCo XML file where this joint will be added
    :return: A list of MuJoCo XML joints, a list of joint parameters
    :raises: RuntimeError: If incorrect number of Coordinates defined for this element
    """

    # Start by parsing the CoordinateSet
    coordinates = parse_coordinates(xml.find("coordinates"))

    # There should be two coordinates for this joint
    if len(coordinates) != 2:
      logger.critical(f"There should be two Coordinates for a UniversalJoint, but {xml.attrib['name']} has "
                      f"{len(coordinates)}")
      raise RuntimeError

    # Collect all joints and params as they will be returned
    all_params = []
    all_joints = []

    first = True
    for coordinate_name in coordinates:
      params = deepcopy(coordinates[coordinate_name])

      # Set default reference position/angle to zero. If this value is not zero, then you need
      # more care while calculating quartic functions for equality constraints. This is not the same as "default_value"
      # for OpenSim Coordinate.
      params["ref"] = 0

      # Both DoFs are rotational, calculate new axes
      params["type"] = "hinge"
      if first:
        axis = estimate_axis(socket_child_frame, np.array([1, 0, 0]))
        first = False
      else:
        axis = estimate_axis(socket_child_frame, np.array([0, 1, 0]))
      params["axis"] = axis

      all_joints.append(etree.SubElement(pointer, "joint", attrib=val2str(filter_keys(params))))
      all_params.append(params)

    return all_joints, all_params