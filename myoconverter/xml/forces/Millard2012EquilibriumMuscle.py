""" Contains the `Millard2012EquilibriumMuscle` parser.

@author: Aleksi Ikkala
"""

import numpy as np

from myoconverter.xml.utils import element_txt2num
from myoconverter.xml.forces.Muscle import Muscle


class Millard2012EquilibriumMuscle(Muscle):
  """ This class parses and converts the OpenSim `Millard2012EquilibriumMuscle` XML element to MuJoCo. """

  def __init__(self, **kwargs):
    super().__init__(**kwargs)

  def _parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Millard2012EquilibriumMuscle` XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Parse time constants
    activation1 = element_txt2num(xml, "activation_time_constant")
    activation2 = element_txt2num(xml, "deactivation_time_constant")

    # Use OpenSim defaults if not defined
    activation1 = 0.01 if np.isnan(activation1) else activation1
    activation2 = 0.04 if np.isnan(activation2) else activation2

    # Estimate activation and deactivation time constants
    self._params["timeconst"] = np.array([activation1, activation2])
