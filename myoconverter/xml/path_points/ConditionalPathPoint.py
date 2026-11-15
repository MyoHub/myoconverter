""" Contains the `ConditionalPathPoint` parser.

@author: Aleksi Ikkala
"""

import os
import matplotlib
import matplotlib.pyplot as pp
import numpy as np
from numpy.polynomial.polynomial import Polynomial
from scipy.spatial.transform import Rotation

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import (
    create_transformation_matrix,
    get_body_and_transform,
    str2vec,
)
from myoconverter.xml.path_points.PathPoint import PathPoint
from myoconverter.xml.path_points.MovingPathPoint import MovingPathPoint
from myoconverter.xml.path_points.utils import get_moving_path_point_dependency
from myoconverter.xml import config as cfg

matplotlib.use("agg")
pp.ioff()


class ConditionalPathPoint(IParser):
  """ This class parses and converts the OpenSim `ConditionalPathPoint` XML element to MuJoCo.

  We anchor the conditional path point either to a same-body `PathPoint` or, when no same-body `PathPoint` exists
  (such as `rect_fem_r-P2` in `Leg6Dof9Musc`, `Gait10dof18musc`, and `Gait2354`), to an adjacent path point on the
  neighboring parent/child body connected across the independent coordinate's joint (#42), and fit a degree-4
  polynomial over the independent coordinate range.
  """

  def __init__(self):
    """ Init method """

    #: PathPoint: A PathPoint object for modelling ConditionalPathPoint
    self.pathpoint = PathPoint()

    #: MovingPathPoint: A MovingPathPoint object for modelling ConditionalPathPoint
    self.moving_pathpoint = MovingPathPoint()

  def parse(self, xml, tendon, force_name, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `ConditionalPathPoint` XML element
    :param tendon: MuJoCo `tendon/spatial` XML element
    :param force_name: Name of the actuator this path point belongs to
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Check if we should treat conditional path points as normal path points
    if cfg.TREAT_AS_NORMAL_PATH_POINT:
      logger.info(f"Treating ConditionalPathPoint {xml.attrib['name']}, as a normal PathPoint")
      self.pathpoint.parse(xml, tendon, force_name, **kwargs)
      return

    # Get socket parent frame and independent coordinate
    socket_parent_frame = xml.find("socket_parent_frame").text
    independent_coordinate = xml.find("socket_coordinate").text.split("/")[-1]
    joint = cfg.M_WORLDBODY.find(f".//joint[@name='{independent_coordinate}']")
    if joint is None:
      logger.warning(
          f"Independent coordinate {independent_coordinate} not found for ConditionalPathPoint "
          f"{xml.attrib['name']}, treating as a normal PathPoint."
      )
      self.pathpoint.parse(xml, tendon, force_name, **kwargs)
      return

    independent_joint_range = str2vec(joint.attrib["range"])
    x_ideal = np.linspace(independent_joint_range[0], independent_joint_range[1], 100)

    # Create an output dir for plots (if it doesn't exist)
    output_dir = os.path.join(cfg.OUTPUT_PLOT_FOLDER, "conditional_path_points")
    os.makedirs(output_dir, exist_ok=True)

    # Get parent (all path_points)
    parent = xml.getparent()
    children = list(parent)
    this_idx = parent.index(xml)

    # Check if previous or next PathPoint exists on the exact same frame
    idx = self._find_previous(children, this_idx - 1, socket_parent_frame)
    if idx == -1:
      idx = self._find_next(children, this_idx + 1, socket_parent_frame)

    if idx >= 0:
      same_body_anchor = True
      anchor_pos = str2vec(children[idx].find("location").text)
      anchor_traj = np.tile(anchor_pos, (len(x_ideal), 1))
    else:
      same_body_anchor = False
      anchor_traj = self._find_cross_body_anchor_traj(
          children, this_idx, socket_parent_frame, independent_coordinate, x_ideal, output_dir
      )

    if anchor_traj is None:
      logger.warning(
          f"Suitable 'anchor' PathPoint was not found for ConditionalPathPoint {xml.attrib['name']}, "
          "treating this as a normal PathPoint."
      )
      self.pathpoint.parse(xml, tendon, force_name, **kwargs)
      return

    cond_pos = str2vec(xml.find("location").text)
    cond_range = np.sort(str2vec(xml.find("range").text))
    active_mask = (x_ideal >= cond_range[0]) & (x_ideal <= cond_range[1])
    if not np.any(active_mask):
      active_mask[np.argmin(np.abs(x_ideal - 0.5 * (cond_range[0] + cond_range[1])))] = True
    rectangular = bool(active_mask[0] == active_mask[-1])

    coord_names = ["x", "y", "z"]
    dependencies = {"x": None, "y": None, "z": None}
    for coord, name in enumerate(coord_names):
      y_ideal = anchor_traj[:, coord].copy()
      y_ideal[active_mask] = cond_pos[coord]

      if same_body_anchor:
        # Use overshoot-free linear (step) or quadratic (rectangular) fit so the conditional point never overshoots
        # past its same-body anchor and folds the tendon back onto itself.
        if rectangular:
          x_points = np.array(
              [
                  independent_joint_range[0],
                  0.5 * (independent_joint_range[0] + independent_joint_range[1]),
                  independent_joint_range[1],
              ]
          )
          ymax = np.max(y_ideal)
          ymin = np.min(y_ideal)
          y_points = np.array([y_ideal[0], ymax if not np.isclose(y_ideal[0], ymax) else ymin, y_ideal[-1]])
          fit = Polynomial.fit(x_points, y_points, 2)
          polycoef = np.zeros((5,))
          converted_coef = fit.convert().coef
          polycoef[: min(5, converted_coef.shape[0])] = converted_coef[:5]
          y_approx = fit(x_ideal)
        else:
          m = (y_ideal[-1] - y_ideal[0]) / (independent_joint_range[1] - independent_joint_range[0])
          c = y_ideal[-1] - m * independent_joint_range[1]
          polycoef = np.array([c, m, 0.0, 0.0, 0.0])
          y_approx = m * x_ideal + c
        dependent_joint_range = np.array([np.min(y_ideal), np.max(y_ideal)])
      else:
        # Cross-body neighbor follows a smooth kinematic arc into cond_pos; fit a degree-4 polynomial
        fit = Polynomial.fit(x_ideal, y_ideal, 4)
        polycoef = np.zeros((5,))
        converted_coef = fit.convert().coef
        polycoef[: min(5, converted_coef.shape[0])] = converted_coef[:5]
        y_approx = fit(x_ideal)
        dependent_joint_range = np.array(
            [min(np.min(y_ideal), np.min(y_approx)), max(np.max(y_ideal), np.max(y_approx))]
        )

      fig = self._plot_figure(
          x_ideal, y_ideal, y_approx, independent_coordinate, xml.attrib["name"], name, rectangular
      )
      pp.savefig(os.path.join(output_dir, f"{xml.attrib['name']}_{name}.svg"))
      pp.close(fig)

      dependencies[name] = (independent_coordinate, polycoef, dependent_joint_range)

    # Treat as a MovingPathPoint
    self.moving_pathpoint.parse(xml, tendon, force_name, dependencies=dependencies, **kwargs)

  def _find_cross_body_anchor_traj(
      self, children, this_idx, socket_parent_frame, independent_coordinate, x_ideal, output_dir
  ):
    """ Compute trajectory `(N, 3)` of an adjacent path point across `x_ideal`, transformed into `socket_parent_frame`. """
    cond_body, T_cond_frame = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, socket_parent_frame)
    T_cond_inv = np.linalg.inv(T_cond_frame)

    neighbor_indices = [i for i in (this_idx + 1, this_idx - 1) if 0 <= i < len(children)]
    for nbr_idx in neighbor_indices:
      nbr = children[nbr_idx]
      nbr_frame_el = nbr.find("socket_parent_frame")
      if nbr_frame_el is None or nbr_frame_el.text is None:
        continue
      try:
        nbr_body, T_nbr_frame = get_body_and_transform(cfg.OPENSIM, cfg.M_WORLDBODY, nbr_frame_el.text)
      except RuntimeError:
        continue

      if nbr_body.getparent() is cond_body:
        moving_body = nbr_body
        nbr_is_child = True
      elif cond_body.getparent() is nbr_body:
        moving_body = cond_body
        nbr_is_child = False
      else:
        continue

      # Ensure moving_body is actuated/coupled by independent_coordinate
      body_joints = moving_body.findall("joint")
      depends_on_coord = False
      for j_el in body_joints:
        j_name = j_el.attrib["name"]
        if j_name == independent_coordinate:
          depends_on_coord = True
          break
        eq_el = cfg.M_EQUALITY.find(f"joint[@joint1='{j_name}']")
        if eq_el is not None and eq_el.attrib.get("joint2") == independent_coordinate:
          depends_on_coord = True
          break
      if not depends_on_coord:
        continue

      # Evaluate neighbor path point coordinates in nbr_body across x_ideal
      p_nbr = self._evaluate_neighbor_point_in_body(
          nbr, T_nbr_frame, independent_coordinate, x_ideal, output_dir
      )
      if p_nbr is None:
        continue

      # Transform across moving_body's base pose and joints for each sample in x_ideal
      b_pos = str2vec(moving_body.attrib.get("pos", "0 0 0"))
      b_quat = str2vec(moving_body.attrib["quat"]) if "quat" in moving_body.attrib else None
      b_euler = str2vec(moving_body.attrib["euler"]) if "euler" in moving_body.attrib else None
      T_base = create_transformation_matrix(pos=b_pos, quat=b_quat, euler=b_euler)

      traj = np.zeros((len(x_ideal), 3))
      for k, q_val in enumerate(x_ideal):
        T_rel = T_base.copy()
        for j_el in body_joints:
          val = self._joint_value_at_q(j_el, independent_coordinate, q_val)
          axis = str2vec(j_el.attrib.get("axis", "0 0 1"))
          j_pos = str2vec(j_el.attrib.get("pos", "0 0 0"))
          if j_el.attrib.get("type", "hinge") == "slide":
            T_j = create_transformation_matrix(pos=axis * val)
          else:
            norm = np.linalg.norm(axis)
            unit_axis = axis / norm if norm > 0 else np.array([0.0, 0.0, 1.0])
            R_j = Rotation.from_rotvec(unit_axis * val).as_matrix()
            T_j = create_transformation_matrix(pos=j_pos - R_j @ j_pos, rotation_matrix=R_j)
          T_rel = np.matmul(T_rel, T_j)

        if nbr_is_child:
          T_cond_from_nbr = np.matmul(T_cond_inv, T_rel)
        else:
          T_cond_from_nbr = np.matmul(T_cond_inv, np.linalg.inv(T_rel))

        traj[k] = (T_cond_from_nbr @ np.append(p_nbr[k], 1.0))[:3]

      return traj

    return None

  def _evaluate_neighbor_point_in_body(self, nbr, T_nbr_frame, independent_coordinate, x_ideal, output_dir):
    """ Evaluate neighbor `PathPoint` or `MovingPathPoint` position `(N, 3)` in `nbr_body` coordinates. """
    if nbr.tag == "MovingPathPoint":
      mpp_dir = os.path.join(cfg.OUTPUT_PLOT_FOLDER, "moving_path_points")
      os.makedirs(mpp_dir, exist_ok=True)
      coords = []
      for loc_tag, sock_tag in (
          ("x_location", "socket_x_coordinate"),
          ("y_location", "socket_y_coordinate"),
          ("z_location", "socket_z_coordinate"),
      ):
        dep_joint, pcoef, _ = get_moving_path_point_dependency(nbr, loc_tag, sock_tag, cfg, mpp_dir)
        poly = Polynomial(pcoef)
        if dep_joint == independent_coordinate:
          coords.append(poly(x_ideal))
        else:
          j_other = cfg.M_WORLDBODY.find(f".//joint[@name='{dep_joint}']") if dep_joint else None
          q_def = float(j_other.attrib.get("user", 0.0)) if j_other is not None else 0.0
          coords.append(np.full_like(x_ideal, poly(q_def)))
      pts = np.column_stack(coords)
    elif nbr.find("location") is not None:
      pts = np.tile(str2vec(nbr.find("location").text), (len(x_ideal), 1))
    else:
      return None

    if not np.allclose(T_nbr_frame, np.eye(4)):
      ones = np.ones((len(x_ideal), 1))
      pts = (np.hstack([pts, ones]) @ T_nbr_frame.T)[:, :3]
    return pts

  def _joint_value_at_q(self, j_el, independent_coordinate, q_val):
    """ Evaluate a MuJoCo joint's value when `independent_coordinate == q_val`. """
    j_name = j_el.attrib["name"]
    if j_name == independent_coordinate:
      return q_val
    eq_el = cfg.M_EQUALITY.find(f"joint[@joint1='{j_name}']")
    if eq_el is not None:
      pcoef = str2vec(eq_el.attrib["polycoef"])
      if eq_el.attrib.get("joint2") == independent_coordinate:
        return float(Polynomial(pcoef)(q_val))
      return float(j_el.attrib.get("user", pcoef[0]))
    return float(j_el.attrib.get("user", 0.0))

  def _find_previous(self, children, idx, socket_parent_frame):
    """ Find previous PathPoint on the same frame. """
    while idx >= 0:
      child = children[idx]
      if child.tag == "PathPoint" and child.find("socket_parent_frame").text == socket_parent_frame:
        break
      idx -= 1
    return idx

  def _find_next(self, children, idx, socket_parent_frame):
    """ Find next PathPoint on the same frame. """
    while idx < len(children):
      child = children[idx]
      if child.tag == "PathPoint" and child.find("socket_parent_frame").text == socket_parent_frame:
        break
      idx += 1
    return idx if idx < len(children) else -1

  def _plot_figure(self, x_ideal, y_ideal, y_approx, independent_joint, dependent_joint, coord, is_rectangular):
    fig = pp.figure(figsize=(10, 8))
    fun_type = "rectangular" if is_rectangular else "step"
    pp.plot(x_ideal, y_ideal, label=f"Ideal {fun_type} function")
    pp.plot(x_ideal, y_approx, label=f"Approximation of the {fun_type} function")
    pp.legend()
    pp.xlabel(f"Independent joint value ({independent_joint})")
    pp.ylabel(f"Dependent joint value ({dependent_joint}_{coord})")
    pp.title(f"Approximation of the {fun_type} function of ConditionalPathPoint {dependent_joint}, {coord} coordinate")
    return fig
