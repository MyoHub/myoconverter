""" Contains the `WrapSphere` parser.

@author: Aleksi Ikkala
"""

from myoconverter.xml.wrap_objects.WrapObject import WrapObject
from myoconverter.xml.utils import str2vec

import numpy as np


class WrapSphere(WrapObject):
  """ This class parses and converts the OpenSim `WrapEllipsoid` XML element to MuJoCo. """

  def _parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `WrapSphere` XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Set type
    self._params["type"] = "sphere"

    # Get orientation
    self._params["euler"] = str2vec(xml.find("xyz_body_rotation").text)

    # Get translation
    self._params["pos"] = str2vec(xml.find("translation").text)

    # Get radius
    self._params["size"] = float(xml.find("radius").text)

    # Calculate sidesite positions
    self._sidesite_pos = \
      {"+x": np.array([self.sidesite_dist()+self._params["size"], 0, 0]),
       "x": np.array([self.sidesite_dist()+self._params["size"], 0, 0]),
       "-x": np.array([-self.sidesite_dist()+self._params["size"], 0, 0]),
       "+y": np.array([0, self.sidesite_dist()+self._params["size"], 0]),
       "y": np.array([0, self.sidesite_dist()+self._params["size"], 0]),
       "-y": np.array([0, -self.sidesite_dist()+self._params["size"], 0]),
       "+z": np.array([0, 0, self.sidesite_dist()+self._params["size"]]),
       "z": np.array([0, 0, self.sidesite_dist()+self._params["size"]]),
       "-z": np.array([0, 0, -self.sidesite_dist()+self._params["size"]])}
