""" This module contains a collection of utility functions useful for parsing and converting the OpenSim model.

@author: Aleksi Ikkala
"""

import numpy as np
from sklearn.metrics import r2_score
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d
from lxml import etree
from numpy.polynomial.polynomial import Polynomial

from loguru import logger


split_name = lambda string: string.split('/')[1:]

def find_element_by_name(xml, names):
  """ Find a name from given XML element.

  :param xml: XML element
  :param names: A list of names
  :return:
  """
  search_str = "".join([f".//*[@name='{name}']" for name in names])
  return xml.find(search_str)

def str2vec(string):
  return np.array(string.split(), dtype=float)

def vec2str(vec):
  return ' '.join(['%.4g' % num for num in vec])

def num2str(x):
  if isinstance(x, (np.ndarray, list)):
    return vec2str(x)
  else:
    return str(x)

def filter_keys(d, prefix="_"):
  """ Filter keys with given prefix from a dictionary

  :param d: A dictionary
  :param prefix: Prefix of keys to be filtered out
  :return: A dictionary without keys starting with `prefix`
  """
  return {k: v for k, v in d.items() if not k.startswith(prefix)}

def filter_nan_values(d):
  """ Filter nan values from dictionary

  :param d: A dictionary
  :return: A dictionary without nan values
  """
  filtered_d = {}
  for k, v in d.items():
    if isinstance(v, (str, bool)):
      filtered_d[k] = v
    elif isinstance(v, (np.ndarray, list)) and np.all(np.isfinite(v)):
      filtered_d[k] = v
    elif np.all(np.isfinite(v)):
      filtered_d[k] = v
  return filtered_d

def filter_set(s, prefix="_"):
  """ Filter elements starting with given prefix from set.

  :param s: A set
  :param prefix: Prefix of elements to be filtered
  :return: A set with elements starting with given prefix filtered
  """
  return {e for e in s if not e.startswith(prefix)}

def val2str(d):
  """ Convert dict values (numbers/vectors/bools) to strings.

  :param d: A dictionary
  :return: Dictionary with numbers/vectors/bools converted to strings
  """
  str_d = {}
  for k, v in d.items():
    if isinstance(v, str):
      str_d[k] = v
    elif isinstance(v, bool):
      str_d[k] = bool2str(v)
    else:
      str_d[k] = num2str(v)
  return str_d

def str2bool(string):
  if isinstance(string, str):
    return True if string in ["true", "True"] else False
  else:
    raise RuntimeError("Input to this function should be a string")

def bool2str(boolean):
  if isinstance(boolean, bool):
    return "true" if boolean else "false"
  else:
    raise RuntimeError("Input to this function should be a boolean")

def create_symmetric_matrix(vec):
  """ Create a symmetric matrix from given upper triangle values.

  :param vec: A vector of upper triangle values (xx,yy,zz,xy,xz,yz)
  :return: A symmetric 3x3 matrix
  """
  # Assume vec is a vector of upper triangle values for matrix of size 3x3 (xx,yy,zz,xy,xz,yz)
  matrix = np.diag(vec[0:3])
  matrix[0, 1] = vec[3]
  matrix[0, 2] = vec[4]
  matrix[1, 2] = vec[5]
  return matrix + matrix.T - np.diag(matrix.diagonal())

def element_txt2num(xml, element_name, default=np.nan):
  """ Convert element to a number, or return `default` if the element is missing.

  :param xml: XML element
  :param element_name: Name of element to be retrieved
  :param default: Default value if element is not found
  :return: Float if element is not found, otherwise `default`
  """
  element = xml.find(element_name)
  return float(element.text) if element is not None else default

def fit_piecewise_linear(x_values, y_values):
  """ Fit a piecewise linear function.

  Useful when we don't need to model the relationship as a quartic function.

  :param x_values: x coordinate values
  :param y_values: y coordinate values
  :return: Piecewise linear model
  """
  return interp1d(x_values, y_values, fill_value="extrapolate")

def fit_spline(x_values, y_values):
  """ Fit a spline, check validity, return polycoef.

  :param x_values: x coordinate values
  :param y_values: y coordinate values
  :return: Spline model, polycoefs, range
  :raises: RunTimeError if not enough x/y coordinate values provided
  """

  if len(x_values) < 2 or len(y_values) < 1:
    logger.critical("Not enough points, can't fit a spline")
    raise RuntimeError

  # Fit a linear / quadratic / cubic / quartic function
  fit = np.polynomial.polynomial.Polynomial.fit(x_values, y_values, min(4, len(x_values) - 1))

  # A simple check to see if the fit is alright (compare to a piecewise linear fit)
  x_hat = np.linspace(np.min(x_values), np.max(x_values), 100)
  y_fit = fit(x_hat)
  pw_lin_fit = interp1d(x_values, y_values)
  y_pw_lin_fit = pw_lin_fit(x_hat)
  if r2_score(y_fit, y_pw_lin_fit) < 0.5 and not np.all(np.isclose(y_fit, y_pw_lin_fit)):
    logger.warning("A bad approximation of a spline")

  # Get the weights
  polycoef = np.zeros((5,))
  polycoef[:fit.convert().coef.shape[0]] = fit.convert().coef

  # Estimate range
  range = np.array([min(y_values), max(y_values)])

  # Make sure range[0] is smaller than range[1]
  if np.isclose(range[0], range[1]):
    range[0] -= 1e-6

  return fit, polycoef, range

def create_transformation_matrix(pos=None, quat=None, rotation_matrix=None, euler=None):
  """ Create a 4x4 transformation matrix.

  Only one of `quat`, `rotation_matrix`, `euler` need to be defined (or none for identity rotation).

  :param pos: Translation vector
  :param quat: Orientation as quaternion
  :param rotation_matrix: Orientation as rotation matrix
  :param euler: Orientation as euler angles
  :return: A 4x4 transformation matrix
  """
  T = np.eye(4)

  if pos is not None:
    T[:3, 3] = pos

  if quat is not None:
    # We assume quat is scalar-first, as mujoco is
    T[:3, :3] = Rotation.from_quat(np.roll(quat, -1)).as_matrix()
  elif rotation_matrix is not None:
    T[:3, :3] = rotation_matrix
  elif euler is not None:
    # The sequence of rotations in mujoco is "XYZ"
    T[:3, :3] = Rotation.from_euler("XYZ", euler).as_matrix()

  return T

def calculate_mujoco_position(tag, name, M_WORLDBODY):
  """ Calculate position of an element in MuJoCo model

  :param tag: Tag of an element
  :param name: Name of an element
  :param M_WORLDBODY: Pointer to MuJoCo XML `worldbody`
  :return: Position of the element in MuJoCo model (when in default pose)
  """

  # Find element
  xml = M_WORLDBODY.find(f".//{tag}[@name='{name}']")

  # Get position (if doesn't exist, use default pos='0 0 0')
  pos = str2vec(xml.attrib.get("pos", "0 0 0"))
  T = np.linalg.inv(create_transformation_matrix(pos=pos))

  # Go upwards in the kinematic tree until root body ("worldbody") and calculate position
  while xml.getparent().tag != "worldbody":
    xml = xml.getparent()
    pos = str2vec(xml.attrib.get("pos", "0 0 0"))
    T_ = create_transformation_matrix(pos=pos)
    T = np.matmul(T, np.linalg.inv(T_))

  T = np.linalg.inv(T)
  return T[:3, 3]

def is_linear(polycoef):
  """ Check if polycoef represents a linear function.

  :param polycoef: Quartic coefficients of a function
  :return: Boolean indicating whether the `polycoef` represents a linear function
  """
  idxs = np.array([0, 2, 3, 4])
  if np.allclose(polycoef[idxs], 0):
    return True
  else:
    return False

def get_body(OPENSIM, M_WORLDBODY, parent_socket_frame):
  """ Get MuJoCo `body` corresponding to and OpenSim `Body`.

  :param OPENSIM: Pointer to OpenSim XML model
  :param M_WORLDBODY: Pointer to MuJoCo XML `worldbody` element
  :param parent_socket_frame: Parent frame socket
  :return: MuJoCo XML element corresponding to `Body`
  :raises: RuntimeError: If a body with given name is not found in the MuJoCo model
  """

  # Make sure the frame name has two elements: first correspond to bodyset, the second to a body name
  names = parent_socket_frame.split("/")[1:]

  # Find opensim frame
  frame = OPENSIM
  for name in names:
    frame = frame.find(f"*[@name='{name}']")
    if frame.tag == "BodySet":
      frame = frame.find("objects")

  # Find the mujoco body; note that there is a special case when the frame may refer to worldbody/ground
  mujoco_body = M_WORLDBODY.find(f".//body[@name='{frame.attrib['name']}']")
  if mujoco_body is None:
    # Check whether the frame refers to worldbody/ground
    if M_WORLDBODY.find(f"geom[@name='{frame.attrib['name']}']") is not None:
      mujoco_body = M_WORLDBODY
    else:
      logger.critical(f"Body {names} not found in MuJoCo model")
      raise RuntimeError

  # Return body
  return mujoco_body

def create_keyframe(MUJOCO, M_WORLDBODY, M_EQUALITY):
  """ Create a keyframe for the MuJoCo model.

  Set keyframe info into MuJoCo model based on joint default values and equality constraints. Default values are
  set as the "user" parameter for each joint in the MuJoCo file. Joint velocity, joint acceleration, and muscle internal
  states are set to zero.

  :param MUJOCO: Pointer to MuJoCo XML model
  :param M_WORLDBODY: Pointer to MuJoCo XML `worldbody` element
  :param M_EQUALITY: Pointer to MuJoCo XML `equality` element
  :return: None
  """

  # Get all joints
  joints = M_WORLDBODY.findall(".//joint")
  if joints is None:
    logger.warning("Could not find any joints for this model! Cannot create a keyframe. ")

  # Create a list of joint names to keep track of indices, and a dict for holding the default values
  joint_names = [joint.attrib["name"] for joint in joints]
  default_values = {}

  # Start from first joint and loop through all of them
  for idx in range(len(joints)):
    _set_default_value(M_EQUALITY, joints, default_values, joint_names, idx)

  # Create the keyframe
  keyframe = etree.SubElement(MUJOCO, "keyframe")
  key = etree.SubElement(keyframe, "key", name="default-pose")

  # Set qpos
  key.attrib["qpos"] = vec2str([default_values[joint_name] for joint_name in joint_names])

def _set_default_value(M_EQUALITY, joints, default_values, joint_names, index):
  """ Set default value for a joint.

  :param M_EQUALITY: Pointer to MuJoCo XML `equality` element
  :param joints: A list of MuJoCo XML `joint` elements
  :param default_values: A dictionary of default values for joints
  :param joint_names: A list of joint names
  :param index: Index of current joint in joints/joint_names
  :return: None
  """

  joint = joints[index]
  joint_name = joint.attrib["name"]

  # If the default value is already set for this joint, continue
  if joint_name in default_values:
    return

  # Use zero by default
  default_values[joint_name] = float(joint.attrib.get("user", 0))

  # Check whether this joint has an equality constraint; overrules previously set default value
  eq = M_EQUALITY.find(f"joint[@joint1='{joint_name}']")

  if eq is not None:

    # Get the coefs
    polycoef = str2vec(eq.attrib["polycoef"])

    # Estimate default value based on polycoef
    if "joint2" in eq.attrib:

      # Create a quadratic polynomial
      fn = Polynomial(polycoef)

      # Make sure joint2 default value has already been set
      _set_default_value(M_EQUALITY, joints, default_values, joint_names, joint_names.index(eq.attrib["joint2"]))

      # Estimate default value for this joint
      default_values[joint_name] = fn(default_values[eq.attrib["joint2"]])

    else:
      default_values[joint_name] = polycoef[0]