""" Contains the `BushingForce` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from loguru import logger

from myoconverter.xml import config as cfg
from myoconverter.xml.forces.TorqueActuator import TorqueActuator
from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import get_body_and_transform, num2str, str2bool, str2vec


class BushingForce(IParser):
  """ This class parses and converts the OpenSim `BushingForce` XML element to MuJoCo. """

  def parse(self, xml):
    """ Map OpenSim `BushingForce` 6-DoF rotational/translational stiffness & damping onto connecting MuJoCo joints.

    :param xml: OpenSim `BushingForce` XML element
    :return: None
    """
    if xml.find("appliesForce") is not None and not str2bool(xml.find("appliesForce").text):
      return

    name = xml.attrib.get("name", "BushingForce")
    frame1_ref = TorqueActuator._read_text(xml, ("socket_frame1", "body_1"))
    frame2_ref = TorqueActuator._read_text(xml, ("socket_frame2", "body_2"))
    if not frame1_ref or not frame2_ref:
      logger.warning(f"BushingForce {name} is missing frame1/frame2 sockets, skipping.")
      return

    try:
      body1, T_f1 = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, frame1_ref)
      body2, T_f2 = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, frame2_ref)
    except RuntimeError:
      logger.warning(f"Could not resolve frames for BushingForce {name}, skipping.")
      return

    is_1_ground = body1 is cfg.M_GROUND or body1 is cfg.M_WORLDBODY
    is_2_ground = body2 is cfg.M_GROUND or body2 is cfg.M_WORLDBODY

    if body2.getparent() is body1 or (is_1_ground and body2.getparent() is cfg.M_WORLDBODY):
      child_body = body2
      R_frame = T_f2[:3, :3]
    elif body1.getparent() is body2 or (is_2_ground and body1.getparent() is cfg.M_WORLDBODY):
      child_body = body1
      R_frame = T_f1[:3, :3]
    else:
      logger.warning(
          f"BushingForce {name} connects non-parent/child bodies ({body1.attrib.get('name')} and "
          f"{body2.attrib.get('name')}); skipping passive joint projection."
      )
      return

    joints = child_body.findall("joint")
    if not joints:
      logger.warning(f"BushingForce {name}: connecting body {child_body.attrib.get('name')} has no joints.")
      return

    k_rot = self._vec_or_zeros(xml, "rotational_stiffness")
    k_trans = self._vec_or_zeros(xml, "translational_stiffness")
    d_rot = self._vec_or_zeros(xml, "rotational_damping")
    d_trans = self._vec_or_zeros(xml, "translational_damping")

    for j_el in joints:
      axis = str2vec(j_el.attrib.get("axis", "0 0 1"))
      norm = np.linalg.norm(axis)
      if norm <= 0:
        continue
      axis_in_frame = R_frame.T @ (axis / norm)
      weights = axis_in_frame ** 2

      if j_el.attrib.get("type", "hinge") == "slide":
        k_add = float(np.dot(weights, k_trans))
        d_add = float(np.dot(weights, d_trans))
      else:
        k_add = float(np.dot(weights, k_rot))
        d_add = float(np.dot(weights, d_rot))

      if k_add != 0.0:
        k_prev = float(j_el.attrib.get("stiffness", 0.0))
        j_el.attrib["stiffness"] = num2str(k_prev + k_add)
        if "springref" not in j_el.attrib and "user" in j_el.attrib:
          j_el.attrib["springref"] = num2str(float(j_el.attrib["user"]))

      if d_add != 0.0:
        d_prev = float(j_el.attrib.get("damping", 0.0))
        j_el.attrib["damping"] = num2str(d_prev + d_add)

  @staticmethod
  def _vec_or_zeros(xml, tag):
    el = xml.find(tag)
    return np.zeros(3) if el is None or not el.text else str2vec(el.text)
