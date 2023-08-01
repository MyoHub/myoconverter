""" Contains the `Ligament` parser.

@author: Aleksi Ikkala
"""

from lxml import etree

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2bool, element_txt2num, val2str, filter_nan_values, filter_keys
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
    params = {"name": f"{xml.attrib['name']}_ligament" }

    # Get resting length
    resting_length = element_txt2num(xml, "resting_length")
    params["springlength"] = resting_length

    # TODO PCSA force probably has something to do with stiffness and damping?
    pcsa_force = element_txt2num(xml, "pcsa_force")

    # Add the tendon to MuJoCo model
    tendon = etree.SubElement(cfg.M_TENDON, "spatial", attrib=val2str(filter_nan_values(filter_keys(params))))

    # Parse path points
    cfg.PATH_POINT_PARSER.parse_all(xml.find("GeometryPath/PathPointSet/objects"), tendon=tendon,
                                force_name=xml.attrib['name'])

    # Parse path wrap set. Note: PathWrapSetParser parses the whole PathWrapSet at once instead of individual objects
    # found in the PathWrapSet. Input muscle name as well, since we want to create muscle-specific wrapping objects for
    # ellipsoids which we can optimize separately later
    cfg.PATH_WRAP_SET_PARSER.parse(xml.find("GeometryPath/PathWrapSet"), tendon=tendon, force_name=xml.attrib["name"])
