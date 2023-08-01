""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `PathWrapSet`

@author: Aleksi Ikkala
"""

import numpy as np
from lxml import etree

from myoconverter.xml.utils import calculate_mujoco_position, vec2str
from myoconverter.xml.wrap_objects.WrapObject import WrapObject


def add_wrapping_site(idx, sites, muscle_name, wrap_object_name, wrap_object_pos, wrap_object_radius, wrap_object_body,
                      wrap_object_body_pos, M_WORLDBODY):
  """ Add wrapping site to given index.

  :param idx: Index indicating between which sites a wrapping site is added
  :param sites: List of MuJoCo sites
  :param muscle_name: Name of the actuator/tendon to which this wrapping site belongs
  :param wrap_object_name: Name of wrapping object
  :param wrap_object_pos: Position of wrapping object
  :param wrap_object_radius: Radius of wrapping object
  :param wrap_object_body: MuJoCo `Body` where wrapping object is located
  :param wrap_object_body_pos: Position of the `wrap_object_body`
  :param M_WORLDBODY: Pointer to MuJoCo XML `worldbody` element
  :return: Dictionary with wrapping site info
  """

  # Get relevant sites
  s1 = sites[idx-1]
  s2 = sites[idx]

  # Check if we need to create a new sidesite
  default_sidesite = wrap_object_body.find(f"site[@name='{wrap_object_name}_sidesite']")

  if default_sidesite is None:
    # Create a new sidesite

    # Estimate distance between segment and wrapping object
    dist, points = segment_distance_to_wrapping_object(s1, s2, wrap_object_pos, M_WORLDBODY)

    # Get relative distance (wrt to wrap object radius)
    rel_dist = dist / wrap_object_radius

    # Get closest distance
    point_idx = np.argmin(rel_dist)

    # If closest point is inside wrap object, estimate a new sidesite position outside wrap object
    if rel_dist[point_idx] < 1:
      v = points[point_idx] - wrap_object_pos
      pos = wrap_object_pos + \
            v * (WrapObject.sidesite_dist() + wrap_object_radius) / np.linalg.norm(v)
    else:
      pos = points[point_idx]

    # Get position relative to wrap object parent body position
    pos = vec2str(pos - wrap_object_body_pos)

  else:

    # Use the position of the default sidesite
    pos = default_sidesite.attrib["pos"]

  # Create the sidesite
  sidesite = etree.Element("site",
                           name=create_sidesite_name(muscle_name, wrap_object_name, idx),
                           pos=pos,
                           size=f"{0.5 * WrapObject.sidesite_dist()}")

  return {"sidesite": sidesite, "wrap_object": wrap_object_name, "wrap_object_body": wrap_object_body,
          "predefined": True}

def maybe_add_wrapping_site(idx, sites, muscle_name, wrap_object_name, wrap_object_pos, wrap_object_radius,
                            wrap_object_body, wrap_object_body_pos, params, M_WORLDBODY):
  """ Maybe add a wrapping site to given index.

  Wrapping site not added if one of the proposed sites is a ConditionalPathPoint or a MovingPathPoint. We can currently
  only add wrapping sites between stationary PathPoints.

  :param idx: Index indicating between which sites a wrapping site is added
  :param sites: List of MuJoCo sites
  :param muscle_name: Name of the actuator/tendon to which this wrapping site belongs
  :param wrap_object_name: Name of wrapping object
  :param wrap_object_pos: Position of wrapping object
  :param wrap_object_radius: Radius of wrapping object
  :param wrap_object_body: MuJoCo `Body` where wrapping object is located
  :param wrap_object_body_pos: Position of the `wrap_object_body`
  :param params: Dictionary with wrapping site info for an existing wrapping site (for this `idx`)
  :param M_WORLDBODY: Pointer to MuJoCo XML `worldbody` element
  :return: Dictionary with wrapping site info, boolean indicating whether a warning should be raised
  """

  # Get relevant sites
  s1 = sites[idx-1]
  s2 = sites[idx]

  # Check if one of the sites is a conditional / moving site
  if M_WORLDBODY.find(f".//*body[@name='{s1.attrib['site']}']") is not None \
      or M_WORLDBODY.find(f".//*body[@name='{s2.attrib['site']}']") is not None:
    return None, True

  # Check if a predefined sidesite already exists here, if so, go to next segment
  if params is not None and params["predefined"]:
    return None, False

  # Estimate distance between segment and wrapping object
  dist, points = segment_distance_to_wrapping_object(s1, s2, wrap_object_pos, M_WORLDBODY)

  # Get relative distance (wrt to wrap object radius)
  rel_dist = dist / wrap_object_radius

  # Get closest distance
  closest_dist = np.min(rel_dist)
  closest_dist_idx = np.argmin(rel_dist)

  # Check if the segment is close enough to wrapping object
  min_rel_dist = 1.5
  if np.any(rel_dist < min_rel_dist):

    # Add a wrapping site; overwrite existing wrapping site if this one is closer to the wrap object
    if params is not None and closest_dist > params["dist"]:
      return None, False

    # Check if a sidesite already exists
    default_sidesite = wrap_object_body.find(f"site[@name='{wrap_object_name}_sidesite']")
    if default_sidesite is None:

      # Find a suitable position for the new sidesite

      # If closest point is inside wrap object, estimate a new sidesite position outside wrap object
      if closest_dist < 1:
        v = points[closest_dist_idx] - wrap_object_pos
        pos = wrap_object_pos + \
              v * (WrapObject.sidesite_dist() + wrap_object_radius) / np.linalg.norm(v)
      else:
        pos = points[closest_dist_idx]

      # Get position relative to wrap object parent body position
      pos = vec2str(pos - wrap_object_body_pos)

    else:

      # Use the position of the default sidesite
      pos = default_sidesite.attrib["pos"]

    # Create the sidesite
    sidesite = etree.Element("site",
                             name=create_sidesite_name(muscle_name, wrap_object_name, idx),
                             pos=pos,
                             size=f"{0.5 * WrapObject.sidesite_dist()}")

    return {"sidesite": sidesite, "wrap_object": wrap_object_name, "wrap_object_body": wrap_object_body,
            "predefined": False, "dist": closest_dist}, False

  else:
    return None, False

def segment_distance_to_wrapping_object(s1, s2, wrap_object_pos, M_WORLDBODY):
  """ Calculate distance between a given segment (straight line between two sites) and a wrapping object.

  :param s1: Start point (site) of the segment
  :param s2: End point (site) of the segment
  :param wrap_object_pos: Position of the wrapping object
  :param M_WORLDBODY: Pointer to MuJoCo XML 'worldbody' element
  :return: Distances between 10 points in the segment and the wrapping object, and positions of those points
  """

  # Get (global) locations of both sites
  s1_pos = calculate_mujoco_position("site", s1.attrib["site"], M_WORLDBODY)
  s2_pos = calculate_mujoco_position("site", s2.attrib["site"], M_WORLDBODY)

  # Calculate vector from s1 to s2
  v = s2_pos - s1_pos

  # Divide the segment from s1 to s2 into 10 points
  k = 10
  points = np.tile(s1_pos, (k, 1)) + np.linspace(0, 1, k).reshape([-1, 1]) * v

  # Calculate distances between the points defined above and the wrap object
  distances = np.linalg.norm(points - wrap_object_pos, axis=1)

  return distances, points

def create_sidesite_name(tendon_name, wrap_object_name, idx):
  """ Utility function to create a name for a {tendon, wrapping object} pair

  :param tendon_name: Name of the tendon that wraps around the wrapping object
  :param wrap_object_name: Wrapping object name
  :param idx: Running index to keep the sidesite names unique
  :return: Unique name for a sidesite
  """
  return f"{wrap_object_name}_{tendon_name}_{idx}_sidesite"
