""" Contains the `Marker` parser.

@author: Aleksi Ikkala
"""

from lxml import etree

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import get_body, vec2str, str2vec
from myoconverter.xml import config as cfg


class Marker(IParser):
  """ This class parses and converts the OpenSim `Marker` XML element to MuJoCo. """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Marker` XML element
    :return: None
    """

    # Get marker name
    name = xml.attrib["name"]

    # Find parent frame
    body = get_body(cfg.OPENSIM, cfg.M_WORLDBODY, xml.find("socket_parent_frame").text)

    # Get location
    location = xml.find("location").text

    # Add marker to mujoco body as a site
    etree.SubElement(body, "site",
                     name=f"{name}_marker",
                     group="4",
                     pos=vec2str(str2vec(location)),  # double conversion to get rid of trailing zeroes
                     rgba="1 0.1 0.5 0.5",
                     size="0.01")