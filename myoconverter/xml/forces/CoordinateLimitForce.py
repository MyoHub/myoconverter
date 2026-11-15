""" Contains the `CoordinateLimitForce` parser.

@author: Aleksi Ikkala
"""

from myoconverter.xml.parsers import IParser

from loguru import logger


class CoordinateLimitForce(IParser):
  """ This class parses and converts the OpenSim `CoordinateLimitForce` XML element to MuJoCo.

  CoordinateLimitForce parser has not been implemented. These forces probably should be optimized after the
  conversion process. I'll leave the old conversion code (from O2MConverter) here for reference, but it is not entirely
  correct, and would require further development.

  Another option to implementing CoordinateLimitForces might be to use MuJoCo tendons, with suitably chosen
  springlengths. The issue with this approach is that there might be multiple tendons for each DoF; would also need
  optimization after conversion process.
  """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `CoordinateLimitForce` XML element
    :return: None
    """

    logger.warning(f"CoordinateLimitForce parsing not implemented, skipping {xml.attrib['name']}")
    return

    # Legacy O2MConverter reference implementation (unimplemented):
    # if force["isDisabled"].lower() == "true":
    #   return
    # joint_name = force["coordinate"]
    # target = None
    # for body in self.joints:
    #   for joint in self.joints[body]:
    #     for mujoco_joint in joint.mujoco_joints:
    #       if mujoco_joint["name"] == joint_name:
    #         target = mujoco_joint
    # assert target is not None, "Cannot set CoordinateLimitForce params, couldn't find the joint"
    # if "range" in target and target["range"][0] != target["range"][1]:
    #   target["limited"] = True
    #   target["user"] = 1
    # stiffness = 0.5 * (float(force["upper_stiffness"]) + float(force["lower_stiffness"]))
    # if stiffness < 1e-4:
    #   damping = float(force["damping"])
    #   if target["motion_type"] == "rotational":
    #     damping *= math.pi / 180
    #   target["damping"] = damping
    # else:
    #   force_coordinate_limits = np.array([float(force["lower_limit"]), float(force["upper_limit"])]) * math.pi / 180
    #   if target["limited"]:
    #     range_val = target.get("range")
    #     width = np.array([force_coordinate_limits[0] - range_val[0], range_val[1] - force_coordinate_limits[1]])
    #     pos_idx = width > 0
    #     if np.any(pos_idx):
    #       target["user"] = 1
    #       target["solimplimit"] = [0.0001, 0.99, np.mean(width[pos_idx])]
    #   else:
    #     width = 0.001
    #     if "range" in target:
    #       width_range = np.array([force_coordinate_limits[0] - target["range"][0],
    #                               target["range"][1] - force_coordinate_limits[1]])
    #       pos_idx = width_range > 0
    #       if np.any(pos_idx):
    #         width = np.mean(width_range[pos_idx])
    #     target["user"] = 1
    #     target["limited"] = True
    #     target["solimplimit"] = [0.0001, 0.99, width, 0.5, 1]
