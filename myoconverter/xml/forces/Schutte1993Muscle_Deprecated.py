""" Contains the `Schutte1993Muscle_Deprecated` parser.

Muscle activation and deactivation linearization by Florian Fischer and Miroslav Bachinski.

@author: Aleksi Ikkala
"""

import numpy as np

from myoconverter.xml.utils import element_txt2num
from myoconverter.xml.forces.Muscle import Muscle


class Schutte1993Muscle_Deprecated(Muscle):
  """ This class parses and converts the OpenSim `Schutte1993Muscle_Deprecated` XML element to MuJoCo. """

  def _parse(self, xml, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Schutte1993Muscle_Deprecated` XML element
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Parse time constants
    activation1 = element_txt2num(xml, "activation1")
    activation2 = element_txt2num(xml, "activation2")

    # Get time scale
    time_scale = element_txt2num(xml, "time_scale", default=0)

    # Estimate activation and deactivation time constants
    if time_scale == 0 or np.isnan(activation1) or np.isnan(activation2):
      self._params["timeconst"] = np.array([activation1, activation2])
    else:
      # Linearize at act=0.5 ctrl=0.5 and calculate activation and deactivation time constants
      act_linearization = ctrl_linearization = 0.5
      time_act = time_scale / ((0.5 + 1.5 * act_linearization) * (activation1 * ctrl_linearization + activation2))
      time_deact = time_scale * (0.5 + 1.5 * act_linearization) / activation2
      self._params["timeconst"] = np.array([time_act, time_deact])
