""" Contains the `CoordinateActuator` parser.

@author: Aleksi Ikkala
"""

from lxml import etree

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import val2str, filter_nan_values, filter_keys
from myoconverter.xml import config as cfg


class CoordinateActuator(IParser):
  """ This class parses and converts the OpenSim `CoordinateActuator` XML element to MuJoCo. """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `CoordinateActuator` XML element
    :return: None
    """

    # Collect params/attributes
    params = {"name": xml.attrib["name"]}

    # Get min/max control
    min_control = xml.find("min_control")
    min_control = "-inf" if min_control is None else min_control.text.lower()
    max_control = xml.find("max_control")
    max_control = "inf" if max_control is None else max_control.text.lower()

    # If either is -inf/inf set ctrllimited=false, not sure if only one can be inf?
    if min_control == "-inf" or max_control == "inf":
      params["ctrllimited"] = False
    else:
      params["ctrllimited"] = True
      params["ctrlrange"] = min_control + " " + max_control

    # Find out which joint is actuated
    params["joint"] = xml.find("coordinate").text

    # Make sure the joint exists
    joint = cfg.M_WORLDBODY.find(f".//joint[@name='{params['joint']}']")
    if joint is None:
      logger.critical(f"Joint {params['joint']} was not found in the converted MuJoCo model, but it is needed for "
                      f"CoordinateActuator {xml.attrib['name']}")

    # TODO how does optimal_force parameter relate to mujoco parameters? gear, gainprm, dynprm, biasprm?
    params["_optimal_force"] = xml.find("optimal_force")

    # Use default motor parameters (gain slightly higher)
    params["class"] = "motor"

    # Add motor to MuJoCo model; must be located before muscles in the xml file
    # Find first muscle element, and add the motor just before that. This keeps the order of the actuators the same
    first_muscle = cfg.M_ACTUATOR.find("muscle")
    idx = len(cfg.M_ACTUATOR.getchildren()) if first_muscle is None else cfg.M_ACTUATOR.index(first_muscle)
    cfg.M_ACTUATOR.insert(idx, etree.Element("motor", attrib=val2str(filter_nan_values(filter_keys(params)))))
