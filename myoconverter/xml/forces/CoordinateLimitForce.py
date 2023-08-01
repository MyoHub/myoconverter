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

    # Ignore disabled forces
    if force["isDisabled"].lower() == "true":
      return

    # Get joint name
    joint_name = force["coordinate"]

    # We need to search for this joint
    target = None
    for body in self.joints:
      for joint in self.joints[body]:
        for mujoco_joint in joint.mujoco_joints:
          if mujoco_joint["name"] == joint_name:
            target = mujoco_joint

    # Check if joint was found
    assert target is not None, "Cannot set CoordinateLimitForce params, couldn't find the joint"

    # TODO for now let's ignore these forces -- they are too difficult to implement and optimize
    # Let's just switch the joint limit on if it's defined; mark this so it won't be unclamped later
    if "range" in target and target["range"][0] != target["range"][1]:
      target["limited"] = True
      target["user"] = 1
      # continue

    # Take the average of stiffness
    stiffness = 0.5 *(float(force["upper_stiffness"]) + float(force["lower_stiffness"]))

    # Stiffness / damping may be defined in two separate forces; we assume that we're dealing with damping
    # if average stiffness is close to zero
    if stiffness < 1e-4:

      # Check if rotational stiffness
      damping = float(force["damping"])
      if target["motion_type"] == "rotational":
        damping *= math.pi/180

      # Set damping
      target["damping"] = damping

    else:

      # We need to create a soft joint coordinate limit, but we can't use separate limits like in OpenSim;
      # this is something we'll need to approximate

      # Limits in CoordinateLimitForce should be in degrees
      force_coordinate_limits = np.array([float(force["lower_limit"]), float(force["upper_limit"])]) * math.pi /180

      # Check if there are hard limits defined for this joint
      if target["limited"]:

        # Range should be given if joint is limited; use range to calculate width param of solimp
        range = target.get("range")
        width = np.array([force_coordinate_limits[0] - range[0], range[1] - force_coordinate_limits[1]])

        # If either width is > 0 create a soft limit
        pos_idx = width > 0
        if np.any(pos_idx):

          # Mark this joint for optimization
          target["user"] = 1

          # Define the soft limit
          target["solimplimit"] = [0.0001, 0.99, np.mean(width[pos_idx])]

      else:

        # Use force_coordinate_limits as range

        # Calculate width with the original range if it was defined
        width = 0.001
        if "range" in target:
          width_range = np.array([force_coordinate_limits[0] - target["range"][0],
                                  target["range"][1] - force_coordinate_limits[1]])
          pos_idx = width_range > 0
          if np.any(pos_idx):
            width = np.mean(width_range[pos_idx])

        # Mark this joint for optimization
        target["user"] = 1

        # Define the soft limit
        target["limited"] = True
        target["solimplimit"] = [0.0001, 0.99, width, 0.5, 1]
