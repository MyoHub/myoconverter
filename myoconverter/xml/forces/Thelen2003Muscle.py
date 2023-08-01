""" Contains the `Thelen2003Muscle` parser.

@author: Aleksi Ikkala
"""

import numpy as np

from myoconverter.xml.utils import element_txt2num
from myoconverter.xml.forces.Muscle import Muscle


class Thelen2003Muscle(Muscle):
  """ This class parses and converts the OpenSim `Thelen2003Muscle` XML element to MuJoCo. """

  def _parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Thelen2003Muscle` XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Parse time constants
    activation1 = element_txt2num(xml, "activation_time_constant")
    activation2 = element_txt2num(xml, "deactivation_time_constant")

    # Use OpenSim defaults if activations not defined
    activation1 = 0.015 if np.isnan(activation1) else activation1
    activation2 = 0.05 if np.isnan(activation2) else activation2

    # Estimate activation and deactivation time constants
    self._params["timeconst"] = np.array([activation1, activation2])
