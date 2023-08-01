""" Contains a higher level `Wrapping Object` parser.

@author: Aleksi Ikkala
"""

from typing import final
from abc import abstractmethod
from lxml import etree
import numpy as np

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import filter_keys, val2str, vec2str, create_transformation_matrix
from myoconverter.xml.wrap_objects.utils import wrap_name_mapping


class WrapObject(IParser):
  """ A parent class to handle common parsing tasks for wrapping objects, like adding MuJoCo geoms. """

  # Sidesites are defined outside wrapping objects (except in WrapTorus)
  _sidesite_dist = 0.001

  def __init__(self, **kwargs):
    """ Init method

    :param kwargs:  Optional keyword arguments -- can be used to pass variables from child classes
    """

    #: dict: Dictionary for parameters. Initialised child objects can overwrite these parameters
    self._params = dict()

    #: dict: Dictionary for sidesite positions
    self._sidesite_pos = dict()

    #: str: Original wrapping object type
    self._osim_type = None

  @classmethod
  def sidesite_dist(cls):
    return cls._sidesite_dist

  @abstractmethod
  def _parse(self, xml, **kwargs):
    """ Wrapping object specific parsers must implement this method.

    Child classes must implement this method. This is called from the 'parse' method below. Add any calculations /
    conversions that weren't included in the default calculations / conversions below.

    :param xml: OpenSim wrapping object XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

  @final
  def parse(self, xml, m_body, **kwargs):
    """ This function handles general parsing and converting of wrapping objects.

    :param xml: OpenSim wrapping object XML element
    :param m_body: MuJoCo body to which this wrapping object belongs to
    :param kwargs: Optional keyword arguments
    :return: None
    :raises: NotImplementedError if an unknown quadrant is defined for this wrapping object
    """

    # Get sidesite quadrant
    self._params["_quadrant"] = xml.find("quadrant").text.lower()
    if self._params["_quadrant"] not in {"all", "+x", "x", "-x", "+y", "y", "-y", "+z", "z", "-z"}:
      logger.error(f"Wrap object {xml.attrib['name']} is defined for quadrant "
                   f"{self._params['_quadrant']}; don't know how to handle this")
      raise NotImplementedError

    # Parse child class specific params
    self._parse(xml, **kwargs)

    # Get osim wrap object type
    self._osim_type = wrap_name_mapping[xml.tag]

    # Insert geom to body
    # Note: wrapping objects don't have mass in OpenSim, so let's define them to belong to group 2
    # Groups 0-1 will be included in mass/inertia calculations, groups 2-5 will not
    etree.SubElement(m_body, "geom",
                     name=f"{xml.attrib['name']}_{self._osim_type}",
                     **val2str(filter_keys(self._params)),
                     rgba="0.19 0.83 0.78 0.2",
                     group="2")

    if self._params["_quadrant"] in self._sidesite_pos:
      # Calculate position of site based on position/rotation of wrapping geom defined above
      T = create_transformation_matrix(pos=self._params["pos"], euler=self._params["euler"])
      site_pos = np.matmul(T, np.concatenate([self._sidesite_pos[self._params["_quadrant"]], np.array([1])]))
      etree.SubElement(m_body, "site",
                       name=f"{xml.attrib['name']}_sidesite",
                       pos=vec2str(site_pos[:3]),
                       size=f"{0.5*WrapObject.sidesite_dist()}")