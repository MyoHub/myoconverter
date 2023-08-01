""" Contains the `PathWrap` parser.

@author: Aleksi Ikkala
"""

import numpy as np

from myoconverter.xml.parsers import IParser
from myoconverter.xml import config as cfg
from myoconverter.xml.utils import calculate_mujoco_position, str2vec
from myoconverter.xml.path_wraps.utils import add_wrapping_site, maybe_add_wrapping_site
from myoconverter.xml.wrap_objects.utils import mujoco_wrap_object_name


class PathWrap(IParser):
  """ This class parses and converts the OpenSim `PathWrap` XML element to MuJoCo.

  This parser should not be called directly, instead the parser in `myoconverter.xml.path_wraps.PathWrapSet` should
  be called.
  """

  def parse(self, xml, tendon, params):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `PathWrap` XML element
    :param tendon: MuJoCo `tendon/spatial` XML element
    :param params: Dictionary of wrapping site parameters
    :return: Boolean indicating whether a warning should be issued
    """

    # Get original wrap object name
    osim_wrap_object_name = xml.find("wrap_object").text

    # Get wrap object name with suffix
    wrap_object_name = mujoco_wrap_object_name(osim_wrap_object_name)

    # Find wrapping object (geom), and the body it belongs to
    geom = cfg.M_WORLDBODY.find(f".//*geom[@name='{wrap_object_name}']")
    wrap_object_body = geom.getparent()

    # Get radius of wrapping object
    wrap_object_radius = str2vec(geom.attrib["size"])[0]

    # Get (global) positions of wrapping object and the body it belongs to
    wrap_object_pos = calculate_mujoco_position("geom", wrap_object_name, cfg.M_WORLDBODY)
    wrap_object_body_pos = calculate_mujoco_position("body", wrap_object_body.attrib["name"], cfg.M_WORLDBODY)

    # Get all path points / sites
    sites = tendon.findall("site")

    # Get muscle name
    muscle_name = xml.getparent().getparent().getparent().getparent().attrib['name']

    # Check range
    if xml.find("range") is not None:
      idxs = str2vec(xml.find("range").text)
    else:
      idxs = np.array([-1, -1])
    idxs = idxs.astype(dtype=np.int16)

    # Issue a warning when adding non-predefined sites and one of the sites is a moving/conditional path point
    issue_warning = False

    if not np.array_equal(idxs, np.array([-1, -1])):

      # if either idxs[0]==-1 or idxs[1]==-1, redefine them to be the first/last site
      if idxs[0] == -1:
        idxs[0] = 1
      if idxs[1] == -1:
        idxs[1] = len(sites)

      # Add all predefined sites
      for idx in range(idxs[0], idxs[1]):
        params[idx] = add_wrapping_site(idx, sites, muscle_name, wrap_object_name,
                                        wrap_object_pos, wrap_object_radius, wrap_object_body, wrap_object_body_pos,
                                        cfg.M_WORLDBODY)

    else:

      # Go through all segments, add wrapping sites where applicable
      for idx in range(1, len(sites)):
        p, issue_warning = maybe_add_wrapping_site(idx, sites, muscle_name, wrap_object_name,
                                                   wrap_object_pos, wrap_object_radius, wrap_object_body,
                                                   wrap_object_body_pos, params.get(idx-1, None), cfg.M_WORLDBODY)

        if p is not None:
          params[idx] = p

    return issue_warning