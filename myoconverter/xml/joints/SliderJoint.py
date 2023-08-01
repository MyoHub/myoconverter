""" Contains the `SliderJoint` parser.

@author: Aleksi Ikkala
"""

from lxml import etree
import numpy as np

from loguru import logger

from myoconverter.xml.joints.Joint import Joint
from myoconverter.xml.joints.utils import parse_coordinates, estimate_axis
from myoconverter.xml.utils import val2str, filter_keys


class SliderJoint(Joint):
  """ This class parses and converts the OpenSim `SliderJoint` XML element to MuJoCo. """

  def _parse(self, xml, socket_parent_frame, socket_child_frame, pointer):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `SliderJoint` XML element
    :param socket_parent_frame: Parent frame socket
    :param socket_child_frame: Child frame socket
    :param pointer: A pointer to the MuJoCo XML file where this joint will be added
    :return: A list of MuJoCo XML joints, a list of joint parameters
    :raises: RuntimeError: Ff multiple Coordinates have been defined for this joint
    """

    # Start by parsing the CoordinateSet
    coordinates = parse_coordinates(xml.find("coordinates"))

    # There should be one coordinate for this joint
    if len(coordinates.keys()) != 1:
      logger.critical(f"There should be only one Coordinate for a SliderJoint, check {xml.attrib['name']}")
      raise RuntimeError
    params = coordinates[next(iter(coordinates))]

    # Set default reference position/angle to zero. If this value is not zero, then you need
    # more care while calculating quartic functions for equality constraints. This is not the same as "default_value"
    # for OpenSim Coordinate.
    params["ref"] = 0

    # We know this is a slide joint with axis [1, 0, 0]
    params["type"] = "slide"

    # Get axis of this transform
    params["axis"] = estimate_axis(socket_child_frame, np.array([1, 0, 0]))

    # Add the joint
    joint = etree.SubElement(pointer, "joint", attrib=val2str(filter_keys(params)))

    return [joint], [params]