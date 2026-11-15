""" Contains the `PointActuator` parser.

@author: Aleksi Ikkala
"""

import numpy as np
from lxml import etree
from scipy.spatial.transform import Rotation

from loguru import logger

from myoconverter.xml import config as cfg
from myoconverter.xml.forces.TorqueActuator import TorqueActuator
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


class PointActuator(IParser):
  """ This class parses and converts the OpenSim `PointActuator` XML element to MuJoCo. """

  def parse(self, xml):
    """ Convert an OpenSim `PointActuator` into a MuJoCo `<motor>` site force transmission.

    :param xml: OpenSim `PointActuator` XML element
    :return: None
    """
    if xml.find("appliesForce") is not None and not str2bool(xml.find("appliesForce").text):
      return

    name = xml.attrib["name"]
    body_ref = TorqueActuator._read_text(xml, ("body", "socket_body"))
    if not body_ref:
      logger.warning(f"PointActuator {name} has no body/socket_body, skipping.")
      return

    try:
      target_body, T_frame = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, body_ref)
    except RuntimeError:
      logger.warning(f"Could not resolve body {body_ref} for PointActuator {name}, skipping.")
      return

    point_el = xml.find("point")
    point = np.zeros(3) if point_el is None or not point_el.text else str2vec(point_el.text)

    point_is_global_el = xml.find("point_is_global")
    point_is_global = False if point_is_global_el is None else str2bool(point_is_global_el.text.strip())

    force_is_global_el = xml.find("force_is_global")
    force_is_global = False if force_is_global_el is None else str2bool(force_is_global_el.text.strip())

    dir_el = xml.find("direction")
    direction = np.array([1.0, 0.0, 0.0]) if dir_el is None or not dir_el.text else str2vec(dir_el.text)
    norm = np.linalg.norm(direction)
    if norm <= 0:
      logger.warning(f"PointActuator {name} has zero-length direction, skipping.")
      return
    direction = direction / norm

    optimal_force = element_txt2num(xml, "optimal_force", default=1.0)
    force_vec = direction * optimal_force

    if point_is_global:
      T_w_gnd = (
          calculate_mujoco_transformation("body", cfg.M_GROUND.attrib["name"], cfg.M_WORLDBODY)
          if cfg.M_GROUND is not None
          else np.eye(4)
      )
      if target_body is cfg.M_WORLDBODY:
        T_w_body = np.eye(4)
      else:
        T_w_body = calculate_mujoco_transformation("body", target_body.attrib["name"], cfg.M_WORLDBODY)
      local_point = (np.linalg.inv(T_w_body) @ T_w_gnd @ np.append(point, 1.0))[:3]
    else:
      local_point = (T_frame @ np.append(point, 1.0))[:3]

    site_name = f"{name}_point_site"
    site_attrib = {"name": site_name, "pos": vec2str(local_point)}
    if not force_is_global and not np.allclose(T_frame[:3, :3], np.eye(3)):
      site_attrib["euler"] = vec2str(Rotation.from_matrix(T_frame[:3, :3]).as_euler("XYZ"))
    etree.SubElement(target_body, "site", attrib=site_attrib)

    params = {"name": name, "site": site_name, "gear": np.concatenate([force_vec, np.zeros(3)])}
    TorqueActuator._parse_control_limits(xml, params)

    if force_is_global:
      ref_name = f"{name}_ground_refsite"
      ref_parent = cfg.M_GROUND if cfg.M_GROUND is not None else cfg.M_WORLDBODY
      etree.SubElement(ref_parent, "site", name=ref_name, pos="0 0 0")
      params["refsite"] = ref_name

    first_muscle = cfg.M_ACTUATOR.find("muscle")
    idx = len(cfg.M_ACTUATOR) if first_muscle is None else cfg.M_ACTUATOR.index(first_muscle)
    cfg.M_ACTUATOR.insert(idx, etree.Element("motor", attrib=val2str(filter_nan_values(filter_keys(params)))))
