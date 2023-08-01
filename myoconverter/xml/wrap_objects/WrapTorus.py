""" Contains the `WrapEllipsoid` parser.

@author: Aleksi Ikkala
"""

import numpy as np

from myoconverter.xml.wrap_objects.WrapObject import WrapObject
from myoconverter.xml.utils import str2vec


class WrapTorus(WrapObject):
  """ This class parses and converts the OpenSim `WrapEllipsoid` XML element to MuJoCo. """

  def _parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `WrapTorus` XML element
    :param kwargs: Optional keyword arguments
    :return:
    """

    # The parse_path_wrap_set in myoconverter.xml.forces.utils assumes that a sidesite exists for a torus wrap object,
    # it should be created here

    # MuJoCo doesn't have a torus wrapping object, we have to approximate with a sphere.

    # Set type
    self._params["type"] = "sphere"

    # Get translation
    self._params["pos"] = str2vec(xml.find("translation").text)

    # Get orientation
    self._params["euler"] = str2vec(xml.find("xyz_body_rotation").text)

    # Get radius
    self._params["size"] = float(xml.find("inner_radius").text)

    # Calculate sidesite positions; the side site only needs to be inside the the object, right?
    self._sidesite_pos = {k: np.array([0, 0, 0]) for k in ["+x", "x", "-x",
                                                           "+y", "y", "-y",
                                                           "+z", "z", "-z", "all"]}
