""" Contains the `PathPoint` parser.

@author: Aleksi Ikkala
"""

from lxml import etree

from myoconverter.xml.parsers import IParser
from myoconverter.xml import config as cfg
from myoconverter.xml.utils import get_body, vec2str


class PathPoint(IParser):
  """ This class parses and converts the OpenSim `PathPoint` XML element to MuJoCo. """

  def parse(self, xml, tendon, force_name, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `MovingPathPoint` XML element
    :param tendon: MuJoCo `tendon/spatial` XML element
    :param force_name: Name of the actuator this path point belongs to
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Get socket parent frame
    socket_parent_frame = xml.find("socket_parent_frame").text

    # Get the mujoco body socket_parent_frame references to
    body = get_body(cfg.OPENSIM, cfg.M_WORLDBODY, socket_parent_frame)

    # Check if pos has been given (e.g. when moving / conditional path point is modelled as a normal path point)
    if "pos" in kwargs:
      pos = vec2str(kwargs["pos"])
    else:
      pos = xml.find("location").text

    # Add this path point to the body referenced in socket parent frame, and to the given tendon.
    # Note: sites may sometimes have non-unique names, have to prepend their names with the muscle name
    etree.SubElement(body, "site", name=f"{force_name}_{xml.attrib['name']}", pos=pos)
    etree.SubElement(tendon, "site", site=f"{force_name}_{xml.attrib['name']}")
