""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `WrapObjectSet`

@author: Aleksi Ikkala
"""

from myoconverter.xml.utils import str2vec
from myoconverter.xml import config as cfg

from loguru import logger

import numpy as np


wrap_name_mapping = {
  "WrapEllipsoid": "ellipsoid",
  "WrapCylinder": "cylinder",
  "WrapTorus": "torus",
  "WrapSphere": "sphere"}

def mujoco_wrap_object_name(osim_wrap_object_name):
  """ Get wrapping object name in MuJoCo

  :param osim_wrap_object_name: Name of an OpenSim wrapping object
  :return: Wrapping object name in MuJoCo model
  :raises: RuntimeError: If a wrap object with given name cannot be found
  """

  # Try first finding the wrap object from BodySet
  wrap_object = cfg.O_BODYSET.findall(f".//*[@name='{osim_wrap_object_name}']")

  # If not found, it could be in the Ground
  if len(wrap_object) == 0:
    wrap_object = cfg.O_GROUND.findall(f".//*[@name='{osim_wrap_object_name}']")

  # We should find only one wrap object
  if len(wrap_object) != 1:
    logger.critical(f"Could not find wrap object '{osim_wrap_object_name}' in the given osim model")
    raise RuntimeError

  return f"{osim_wrap_object_name}_{wrap_name_mapping[wrap_object[0].tag]}"

def projected_point_inside_segment(a, b, p):
  """ Check if projection of point p is inside segment starting from a and ending in b.

  Adapted from https://stackoverflow.com/a/47492642/6065074

  :param a: Start point of segment
  :param b: End point of segment
  :param p: Point
  :return: Boolean indicating whether projection of point p is inside given segment
  """
  delta = b - a
  inner_product = np.dot(p-a, delta)
  return inner_product >= 0 and inner_product <= np.dot(delta, delta)

def point_distance_from_segment(a, b, p):
  """ Calculate distance between point p and segment starting from a and ending in b.

  Adapted from https://www.nagwa.com/en/explainers/939127418581/

  :param a: Start point of segment
  :param b: End point of segment
  :param p: Point
  :return: DIstance between point p and segment
  """
  return np.linalg.norm(np.cross(p-a, b-a)) / np.linalg.norm(b-a)

def find_wrap_path(xml, body, tendon, center):
  """ Find locations of wrapping sites within a tendon. This placement is very much based on heuristics, and probably
  could be improved.

  NOTE! This function is not used anywhere. Leaving it here in case it could be useful in the future.

  :param xml: OpenSim wrapping object XML element
  :param body: MuJoCo `body` XML element
  :param tendon: MuJoCo `tendon/spatial` XML element
  :param center: Position of wrapping object
  :return: None or a list of indices
  :raises: RuntimeError: If something goes wrong while estimating which wrap object is closest
  """

  # 1. For all consecutive sites, calculate whether projected center of wrapping object falls within the segment
  #   1.1 If the center falls onto one segment, put the wrapping object there
  #   1.2 If the center falls onto multiple segments, calculate distances between center and segment lines, place
  #       wrapping object to segment that it's closest to
  #   1.3 If the center falls on zero segments, calculate distances between center and segment start/end points
  #     1.3.1 If there exists a site before/after the nearest start/end point, put wrapping object there
  #     1.3.2 otherwise issue warning and put wrapping object onto nearest segment

  # We ignore moving / conditional sites, since they are defined in a different body ("imaginary" body) so that
  # they can move. It is very difficult to estimate the locations of those sites

  # Find sites in the tendon, and their locations in the body. Note: only sites defined in the body where wrapping
  # object is defined in
  tendon_site_locations = []
  tendon_site_names = []
  tendon_site_idxs = []
  tendon_children = tendon.getchildren()
  for child_idx, tendon_child in enumerate(tendon_children):

    # We're only interested in sites
    if tendon_child.tag != "site":
      continue

    # Is this site in the body where wrapping object is defined?
    body_site = body.find(f"site[@name='{tendon_child.attrib['site']}']")
    if body_site is not None:
      tendon_site_names.append(tendon_child.attrib["site"])
      tendon_site_idxs.append(child_idx)
      tendon_site_locations.append(str2vec(body_site.attrib["pos"]))

  if len(tendon_site_idxs) == 0:
    logger.warning(f"Could not find any sites for tendon {tendon.attrib['name']} that are located in body "
          f"{body.attrib['name']}. Could be because of conditional or moving path points (sites) that are defined "
          f"in other bodies. Could not add wrapping body {xml.attrib['name']}, you need to do it yourself. ")
    return

  # Check whether projected center of wrapping object falls within the segments (segments defined by consecutive sites)
  # Calculate also min distance from start/end points of segments to center, and min distance between center and line
  # defined by each segment
  inside = []
  min_dist_from_endpoints = np.zeros((len(tendon_site_idxs)-1,2))
  segment = np.empty((len(tendon_site_idxs)-1,2), dtype=int)

  for seg_idx in range(len(tendon_site_idxs)-1):

    # Get start and end point of this particular path
    s1 = tendon_site_locations[seg_idx]
    s2 = tendon_site_locations[seg_idx+1]

    # Calculate whether projected center of the wrapping object is within the segment
    if projected_point_inside_segment(s1, s2, center):
      inside.append(seg_idx)

    # Calculate minimum distance between start and end points and the wrapping object center
    min_dist_from_endpoints[seg_idx] = [np.linalg.norm(s1 - center), np.linalg.norm(s2 - center)]

    # Get site indices of start and end point
    segment[seg_idx] = [tendon_site_idxs[seg_idx], tendon_site_idxs[seg_idx+1]]

  if len(inside) > 0:
    # Wrap each segment, into which the projected center falls, around the object
    idxs = []
    for seg_idx in inside:
      for idx in range(segment[seg_idx][0], segment[seg_idx][1]):
        # Make sure tendon doesn't already wrap around an object in this segment
        if tendon_children[idx].tag == "site" or tendon_children[idx+1].tag == "site":
          idxs.append(idx+1)

  # Otherwise there's only one site defined in this body, put wrapping object after that site (or before if the site
  # is last one)
  else:

    logger.warning(f"Could not place wrapping sites for tendon {tendon.attrib['name']} reliably. You might want to "
          f"check the site definitions yourself.")

    # Find site closest to center
    dist_to_center = [np.linalg.norm(site_loc-center) for site_loc in tendon_site_locations]

    # Check if this site is part of a segment (might not be if only 1 site found in this body)
    idx = tendon_site_idxs[np.argmin(dist_to_center)]
    loc = list(zip(*np.where(segment==idx)))
    if len(loc) > 1:
      logger.critical("Something's wrong, center should be close to only one site")
      raise RuntimeError

    # If site is part of a segment, use it to figure out where we should place the wrap site
    if len(loc) > 0:
      loc = loc[0]

      if loc[1] == 0:
        # idx cannot be 0, because a wrapping site cannot start a tendon
        if idx == 0:
          idx += 1
      elif loc[1] == 1:
        # Is there a subsequent site in another body?
        if idx < len(tendon_children) - 1 and tendon_children[idx+1].tag == "site":
          idx = idx + 1

    else:

      # Put the wrap site after the site that is closest (unless it's the last site in this tendon)
      idx = tendon_site_idxs[0] + 1

      # If this site is the last one in the tendon, we have to move the wrapping site
      # Or, if the next child in tendon_children is not a site (but the current is), we have to move also
      if idx >= len(tendon_children)-1 or (tendon_children[idx].tag == "site" and tendon_children[idx+1].tag != "site"):
        # Let's try to put the wrapping site before the found site
        if idx > 1:
          idx = idx-1

    # Return as a list
    if tendon_children[idx-1].tag == "site" and tendon_children[idx].tag == "site":
      idxs = [idx]
    else:
      idxs = []

  return idxs