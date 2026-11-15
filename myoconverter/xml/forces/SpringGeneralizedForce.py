""" Contains the `SpringGeneralizedForce` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from loguru import logger

from myoconverter.xml import config as cfg
from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import element_txt2num, num2str, str2bool


class SpringGeneralizedForce(IParser):
  """ This class parses and converts the OpenSim `SpringGeneralizedForce` XML element to MuJoCo. """

  def parse(self, xml):
    """ Map OpenSim `SpringGeneralizedForce` (`coordinate`, `stiffness`, `rest_length`, `viscosity`) onto a MuJoCo joint.

    :param xml: OpenSim `SpringGeneralizedForce` XML element
    :return: None
    """
    if xml.find("appliesForce") is not None and not str2bool(xml.find("appliesForce").text):
      return

    coord_el = xml.find("coordinate")
    if coord_el is None or not coord_el.text:
      logger.warning(f"SpringGeneralizedForce {xml.attrib.get('name', '')} has no <coordinate>, skipping.")
      return

    coord_name = coord_el.text.strip().split("/")[-1]
    joint = cfg.M_WORLDBODY.find(f".//joint[@name='{coord_name}']")
    if joint is None:
      logger.warning(
          f"Joint {coord_name} not found in MuJoCo model for SpringGeneralizedForce {xml.attrib.get('name', '')}."
      )
      return

    stiffness = element_txt2num(xml, "stiffness", default=0.0)
    rest_length = element_txt2num(xml, "rest_length", default=0.0)
    viscosity = element_txt2num(xml, "viscosity", default=0.0)

    k_old = float(joint.attrib.get("stiffness", 0.0))
    ref_old = float(joint.attrib.get("springref", 0.0))
    k_total = k_old + stiffness

    if np.isfinite(k_total) and k_total > 0:
      eq_ref = (k_old * ref_old + stiffness * rest_length) / k_total
      joint.attrib["stiffness"] = num2str(k_total)
      joint.attrib["springref"] = num2str(eq_ref)
    elif stiffness != 0.0:
      joint.attrib["stiffness"] = num2str(k_total)
      joint.attrib["springref"] = num2str(rest_length)

    if np.isfinite(viscosity) and viscosity != 0.0:
      d_old = float(joint.attrib.get("damping", 0.0))
      joint.attrib["damping"] = num2str(d_old + viscosity)
