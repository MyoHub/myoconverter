""" Contains the `TorqueActuator` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from lxml import etree
from scipy.spatial.transform import Rotation

from loguru import logger

from myoconverter.xml import config as cfg
from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import (
    calculate_mujoco_transformation,
    element_txt2num,
    filter_keys,
    filter_nan_values,
    get_body_and_transform,
    str2bool,
    str2vec,
    val2str,
    vec2str,
)


class TorqueActuator(IParser):
  """ This class parses and converts the OpenSim `TorqueActuator` XML element to MuJoCo (#24). """

  def parse(self, xml):
    """ Convert an OpenSim `TorqueActuator` into a MuJoCo `<motor>` site/refsite transmission.

    :param xml: OpenSim `TorqueActuator` XML element
    :return: None
    """
    if xml.find("appliesForce") is not None and not str2bool(xml.find("appliesForce").text):
      return

    name = xml.attrib["name"]

    body_a_ref = self._read_text(xml, ("bodyA", "socket_bodyA"))
    if not body_a_ref:
      logger.warning(f"TorqueActuator {name} has no bodyA/socket_bodyA, skipping.")
      return
    body_b_ref = self._read_text(xml, ("bodyB", "socket_bodyB"), default="ground")

    try:
      body_a, T_frame_a = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, body_a_ref)
      body_b, T_frame_b = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, body_b_ref)
    except RuntimeError:
      logger.warning(f"Could not resolve bodyA/bodyB for TorqueActuator {name}, skipping.")
      return

    torque_is_global_el = xml.find("torque_is_global")
    torque_is_global = True if torque_is_global_el is None else str2bool(torque_is_global_el.text.strip())

    axis_el = xml.find("axis")
    axis = np.array([0.0, 0.0, 1.0]) if axis_el is None or not axis_el.text else str2vec(axis_el.text)
    norm = np.linalg.norm(axis)
    if norm <= 0:
      logger.warning(f"TorqueActuator {name} has zero-length axis, skipping.")
      return
    axis = axis / norm

    optimal_force = element_txt2num(xml, "optimal_force", default=1.0)
    torque_vec = axis * optimal_force

    is_b_ground = body_b is cfg.M_GROUND or body_b is cfg.M_WORLDBODY

    site_a_name = f"{name}_torque_siteA"
    site_b_name = f"{name}_torque_siteB"

    params = {"name": name}
    self._parse_control_limits(xml, params)

    if torque_is_global and is_b_ground:
      # Express torque in OpenSim Ground frame (cfg.M_GROUND); +T on bodyA, -T on ground
      etree.SubElement(body_a, "site", name=site_a_name, pos="0 0 0")
      ref_parent = cfg.M_GROUND if cfg.M_GROUND is not None else cfg.M_WORLDBODY
      etree.SubElement(ref_parent, "site", name=site_b_name, pos="0 0 0")
      params["site"] = site_a_name
      params["refsite"] = site_b_name
      params["gear"] = np.concatenate([np.zeros(3), torque_vec])

    elif not torque_is_global and is_b_ground:
      # Express torque in bodyA frame (including PhysicalOffsetFrame rotation if any); +T on bodyA, -T on ground
      site_attrib = {"name": site_a_name, "pos": "0 0 0"}
      if not np.allclose(T_frame_a[:3, :3], np.eye(3)):
        site_attrib["euler"] = vec2str(Rotation.from_matrix(T_frame_a[:3, :3]).as_euler("XYZ"))
      etree.SubElement(body_a, "site", attrib=site_attrib)
      params["site"] = site_a_name
      params["gear"] = np.concatenate([np.zeros(3), torque_vec])

    elif not torque_is_global and not is_b_ground:
      # Put refsite on bodyA and site on bodyB with negated gear:
      # MuJoCo expresses gear in refsite (bodyA), applies -T to site (bodyB) and -(-T) = +T to refsite (bodyA)!
      ref_attrib = {"name": site_a_name, "pos": "0 0 0"}
      if not np.allclose(T_frame_a[:3, :3], np.eye(3)):
        ref_attrib["euler"] = vec2str(Rotation.from_matrix(T_frame_a[:3, :3]).as_euler("XYZ"))
      etree.SubElement(body_a, "site", attrib=ref_attrib)
      etree.SubElement(body_b, "site", name=site_b_name, pos="0 0 0")
      params["site"] = site_b_name
      params["refsite"] = site_a_name
      params["gear"] = np.concatenate([np.zeros(3), -torque_vec])

    else:
      # torque_is_global=True and bodyB != ground: orient refsite on bodyB to align with OpenSim Ground in default pose
      etree.SubElement(body_a, "site", name=site_a_name, pos="0 0 0")
      T_w_gnd = (
          calculate_mujoco_transformation("body", cfg.M_GROUND.attrib["name"], cfg.M_WORLDBODY)
          if cfg.M_GROUND is not None
          else np.eye(4)
      )
      T_w_b = calculate_mujoco_transformation("body", body_b.attrib["name"], cfg.M_WORLDBODY)
      R_b_gnd = np.matmul(T_w_b[:3, :3].T, T_w_gnd[:3, :3])
      ref_attrib = {"name": site_b_name, "pos": "0 0 0"}
      if not np.allclose(R_b_gnd, np.eye(3)):
        ref_attrib["euler"] = vec2str(Rotation.from_matrix(R_b_gnd).as_euler("XYZ"))
      etree.SubElement(body_b, "site", attrib=ref_attrib)
      params["site"] = site_a_name
      params["refsite"] = site_b_name
      params["gear"] = np.concatenate([np.zeros(3), torque_vec])

    first_muscle = cfg.M_ACTUATOR.find("muscle")
    idx = len(cfg.M_ACTUATOR) if first_muscle is None else cfg.M_ACTUATOR.index(first_muscle)
    cfg.M_ACTUATOR.insert(idx, etree.Element("motor", attrib=val2str(filter_nan_values(filter_keys(params)))))

  @staticmethod
  def _read_text(xml, tags, default=None):
    for tag in tags:
      el = xml.find(tag)
      if el is not None and el.text is not None and el.text.strip():
        return el.text.strip()
    return default

  @staticmethod
  def _parse_control_limits(xml, params):
    min_el = xml.find("min_control")
    max_el = xml.find("max_control")
    min_val = -np.inf if min_el is None or not min_el.text else float(min_el.text)
    max_val = np.inf if max_el is None or not max_el.text else float(max_el.text)
    if not np.isfinite(min_val) or not np.isfinite(max_val) or min_val > max_val:
      params["ctrllimited"] = False
    else:
      params["ctrllimited"] = True
      if np.isclose(min_val, max_val):
        params["ctrlrange"] = f"{min_val - 1e-6:.6g} {max_val + 1e-6:.6g}"
      else:
        params["ctrlrange"] = f"{min_val:.6g} {max_val:.6g}"
