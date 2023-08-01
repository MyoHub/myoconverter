""" Contains the `PointActuator` parser.

@author: Aleksi Ikkala
"""

from myoconverter.xml.parsers import IParser

from loguru import logger


class TorqueActuator(IParser):
  """ This class parses and converts the OpenSim `TorqueActuator` XML element to MuJoCo (not implemented yet). """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `TorqueActuator` XML element
    :return: None
    """
    logger.warning(f"TorqueActuator parser has not been implemented, skipping {xml.attrib['name']}")