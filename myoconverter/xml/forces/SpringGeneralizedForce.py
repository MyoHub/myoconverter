""" Contains the `SpringGeneralizedForce` parser.

@author: Aleksi Ikkala
"""

from myoconverter.xml.parsers import IParser

from loguru import logger


class SpringGeneralizedForce(IParser):
  """ This class parses and converts the OpenSim `SpringGeneralizedForce` XML element to MuJoCo (not implemented yet).
  """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `SpringGeneralizedForce` XML element
    :return: None
    """
    logger.warning(f"SpringGeneralizedForce parser has not been implemented, skipping {xml.attrib['name']}")