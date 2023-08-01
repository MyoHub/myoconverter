""" Contains the `UniversalJoint` parser.

@author: Aleksi Ikkala
"""

from myoconverter.xml.parsers import IParser


class WeldJoint(IParser):
  """ This class parses and converts the OpenSim `UniversalJoint` XML element to MuJoCo.

  MuJoCo connects two bodies together as weld joints, if there are no joints defined.
  """

  def parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting -- which is no-op in this case.

    :param xml: OpenSim `WeldJoint` XML element
    :param kwargs: Optional keyword arguments
    :return: A list of MuJoCo XML joints, a list of joint parameters
    """
    return [], [dict()]