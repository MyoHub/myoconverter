""" Contains a higher level `Muscle` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from typing import final
from abc import abstractmethod
from lxml import etree

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2bool, element_txt2num, val2str, filter_nan_values, str2vec
from myoconverter.xml.forces.utils import calculate_length_range
from myoconverter.xml import config as cfg


class Muscle(IParser):
  """ A parent class to handle common parsing tasks for muscle forces (actuators), like parsing path points.

  Note that we set only (arbitrary) default parameters for the muscles here, as they will be optimized later.
  """

  def __init__(self, **kwargs):
    """ Init method

    :param kwargs: Optional keyword arguments -- used to pass variables from child classes
    """

    #: dict: Dictionary for parameters. Initialised child objects can overwrite these parameters
    self._params = dict()


  @abstractmethod
  def _parse(self, xml, **kwargs):
    """ Muscle-specific parsers must implement this method.

    Child classes must implement this method. This is called from the 'parse' method below. Add any calculations /
    conversions that weren't included in the default calculations / conversions below.

    :param xml: OpenSim muscle actuator XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

  @final
  def parse(self, xml, **kwargs):
    """ This function handles general parsing and converting of muscle actuators.

    Note that we set only (arbitrary) default parameters for the muscles here, as they will be optimized later.

    :param xml: OpenSim muscle actuator XML element
    :param kwargs: Optional keyword arguments
    :return: None
    :raises: RuntimeError: If default values for 'muscle' class cannot be found in the MuJoCo model
    """

    # Do some default parsing; most of these seem to apply to all muscles

    # If this actuator is not enabled, return
    if xml.find("appliesForce") is not None and not str2bool(xml.find("appliesForce").text):
      return

    # Collect attributes in a dict
    self._params["name"] = xml.attrib["name"]

    # Use a larger scale ratio than literature suggests, e.g. Garner and Pandy (2002) uses [0.5, 1.5]
    # (https://web.ecs.baylor.edu/faculty/garner/Research/GarnerPandy2003ParamEst.pdf)
    #self._params["range"] = np.array([0.5, 2])

    # Get optimal fiber length, tendon slack length, and pennation angle
    optimal_fiber_length = element_txt2num(xml, "optimal_fiber_length")
    tendon_slack_length = element_txt2num(xml, "tendon_slack_length")
    pennation_angle = element_txt2num(xml, "pennation_angle_at_optimal")

    # Try to optimize length range
    #self._params["lengthrange"] = calculate_length_range(self._params["range"], optimal_fiber_length,
    #                                                     tendon_slack_length, pennation_angle)
    # Use an arbitrary value for length range
    self._params["lengthrange"] = np.array([0.01, 1])

    # Let's use max isometric force as an approximation for peak active force at rest
    self._params["force"] = xml.find("max_isometric_force").text

    # Parse control limits; note that mujoco clamps control values internally to [0, 1], so we'll use that as default
    # (see https://mujoco.readthedocs.io/en/stable/modeling.html#cmuscle); this default is set in the "template.xml"
    # file
    muscle_defaults = cfg.MUJOCO.find(".//default[@class='muscle']/muscle")
    if muscle_defaults is None or "ctrlrange" not in muscle_defaults.attrib:
      logger.error("Could not find required defaults for class 'muscle' in the mujoco file")
      raise RuntimeError
    m_ctrlrange = str2vec(muscle_defaults.attrib["ctrlrange"])

    # Get control range from the osim file, if it is defined
    o_ctrlrange = np.array([element_txt2num(xml, "min_control"), element_txt2num(xml, "max_control")])

    # Update ctrlrange if it is not the same as the default
    finite = np.isfinite(o_ctrlrange)
    if np.any(finite):
      if not np.allclose(o_ctrlrange[finite], m_ctrlrange[finite]):
        self._params["ctrlrange"] = m_ctrlrange
        self._params["ctrlrange"][finite] = o_ctrlrange[finite]

    # Add tendon and class
    self._params["tendon"] = self._params["name"] + "_tendon"
    self._params["class"] = "muscle"

    # At this point do child class specific parsing
    self._parse(xml, **kwargs)

    # Add the muscle to MuJoCo model
    etree.SubElement(cfg.M_ACTUATOR, "muscle", attrib=val2str(filter_nan_values(self._params)))

    # Add the tendon to MuJoCo model
    tendon = etree.SubElement(cfg.M_TENDON, "spatial", name=self._params["tendon"])

    # Parse path points; input muscle name as well, since some of the site names may be non-unique, so we should prepend
    # their names with the name of the muscle
    cfg.PATH_POINT_PARSER.parse_all(xml.find("GeometryPath/PathPointSet/objects"), tendon=tendon,
                                force_name=xml.attrib["name"])

    # Parse path wrap set. Note: PathWrapSetParser parses the whole PathWrapSet at once instead of individual objects
    # found in the PathWrapSet. Input muscle name as well, since we want to create muscle-specific wrapping objects for
    # ellipsoids which we can optimize separately later
    cfg.PATH_WRAP_SET_PARSER.parse(xml.find("GeometryPath/PathWrapSet"), tendon=tendon, force_name=xml.attrib["name"])
