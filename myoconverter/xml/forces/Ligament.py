""" Contains the `Ligament` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from lxml import etree

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import (
    element_txt2num,
    filter_keys,
    filter_nan_values,
    str2bool,
    str2vec,
    val2str,
)
from myoconverter.xml import config as cfg


class Ligament(IParser):
  """ This class parses and converts the OpenSim `Ligament` XML element to MuJoCo. """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Ligament` XML element
    :return: None
    """

    # If this actuator is not enabled, return
    if xml.find("appliesForce") is not None and not str2bool(xml.find("appliesForce").text):
      return

    # Collect attributes in a dict
    params = {"name": f"{xml.attrib['name']}_ligament"}

    # Get resting length and PCSA force to derive linear tendon stiffness
    resting_length = element_txt2num(xml, "resting_length")
    params["springlength"] = resting_length

    pcsa_force = element_txt2num(xml, "pcsa_force")
    if np.isfinite(pcsa_force) and np.isfinite(resting_length) and resting_length > 0 and pcsa_force >= 0:
      norm_slope = self._estimate_curve_slope(xml.find("force_length_curve"))
      params["stiffness"] = norm_slope * pcsa_force / resting_length

    # Add the tendon to MuJoCo model
    tendon = etree.SubElement(cfg.M_TENDON, "spatial", attrib=val2str(filter_nan_values(filter_keys(params))))

    # Parse path points
    cfg.PATH_POINT_PARSER.parse_all(
        xml.find("GeometryPath/PathPointSet/objects"),
        tendon=tendon,
        force_name=xml.attrib["name"],
    )

    # Parse path wrap set
    cfg.PATH_WRAP_SET_PARSER.parse(
        xml.find("GeometryPath/PathWrapSet"),
        tendon=tendon,
        force_name=xml.attrib["name"],
    )

  @staticmethod
  def _estimate_curve_slope(curve_xml):
    """ Estimate normalized tensile slope `d(F/F_pcsa)/d(strain)` from `force_length_curve` if present. """
    if curve_xml is None or len(curve_xml) == 0:
      return 1.0
    fn = curve_xml[0]
    x_el = fn.find("x")
    y_el = fn.find("y")
    if x_el is None or y_el is None or not x_el.text or not y_el.text:
      return 1.0
    x = str2vec(x_el.text)
    y = str2vec(y_el.text)
    pos_mask = (x > 0) & (y > 0)
    if np.any(pos_mask):
      slopes = y[pos_mask] / x[pos_mask]
      slope = float(np.median(slopes))
      if np.isfinite(slope) and slope > 0:
        return slope
    return 1.0
