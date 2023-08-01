""" Contains the `WrapCylinder` parser.

@author: Aleksi Ikkala
"""

from myoconverter.xml.utils import str2vec
from myoconverter.xml.wrap_objects.WrapObject import WrapObject

import numpy as np


class WrapCylinder(WrapObject):
  """ This class parses and converts the OpenSim `WrapCylinder` XML element to MuJoCo. """

  def _parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `WrapCylinder` XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Set type
    self._params["type"] = "cylinder"

    # Get orientation
    self._params["euler"] = str2vec(xml.find("xyz_body_rotation").text)

    # Get translation
    self._params["pos"] = str2vec(xml.find("translation").text)

    # Get dimensions
    self._params["size"] = np.array([float(xml.find("radius").text), float(xml.find("length").text)/2])

    # Calculate sidesite positions
    self._sidesite_pos = \
      {"+x": np.array([self.sidesite_dist()+self._params["size"][0], 0, 0]),
       "x": np.array([self.sidesite_dist()+self._params["size"][0], 0, 0]),
       "-x": np.array([-(self.sidesite_dist()+self._params["size"][0]), 0, 0]),
       "+y": np.array([0, self.sidesite_dist()+self._params["size"][0], 0]),
       "y": np.array([0, self.sidesite_dist()+self._params["size"][0], 0]),
       "-y": np.array([0, -(self.sidesite_dist()+self._params["size"][0]), 0]),
       "+z": np.array([0, 0, self.sidesite_dist()+self._params["size"][1]]),
       "z": np.array([0, 0, self.sidesite_dist()+self._params["size"][1]]),
       "-z": np.array([0, 0, -(self.sidesite_dist()+self._params["size"][1])])}
