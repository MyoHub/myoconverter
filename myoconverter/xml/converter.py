""" This module contains the OpenSim-to-MuJoCo model (XML file) convertion main access points.

You can convert an OpenSim XML model file to MuJoCo by calling the :py:func:`convert` function, or by running this
module.

Example:

  in a command shell::

    python converter.py /path/to/opensim/model.xml /output/directory/for/mujoco/model/


  or, in a Python file::

    from myoconverter.xml.converter import convert
    convert(xml_file, output_folder, **kwargs)

"""

import argparse
from loguru import logger
from lxml import etree

from myoconverter.xml import config as cfg
from myoconverter.xml.utils import find_element_by_name, split_name, create_keyframe


def convert(xml_file, output_folder, **kwargs):
  """ Convert an OpenSim XML model file to MuJoCo XMl model file.

  :param xml_file: Path to the OpenSim XML model file
  :param output_folder: Path to folder where converted model is saved
  :param kwargs: geometry_folder (str), add_ground_geom (bool), treat_as_normal_path_point (bool)
  :return: Path to the MuJoCo XML file
  """

  # First, initialise config (read and parse OpenSim and MuJoCo XML files, set variables etc.)
  cfg.initialise(xml_file, output_folder, **kwargs)

  logger.info(f"Commencing the conversion procedure!")

  # Parse ground. The ground may have attached geometries and wrapping objects
  _parse_ground()

  # Parse constraints. The constraints need to be parsed before joints, because we might need to update the polycoefs
  # of some joint constraints when parsing joints
  _parse_constraints()

  # Parse bodies and joints
  _parse_bodies_and_joints()

  # Parse forces
  _parse_forces()

  # Parse markers -- needed for later optimization procedures
  _parse_markers()

  # Set keyframe
  _set_keyframe()

  # Copy credits from the OpenSim model
  _copy_credits()

  # Finally, save the MuJoCo model into XML file
  logger.info(f"Writing the converted model into {cfg.OUTPUT_XML_FILE}")
  etree.ElementTree(cfg.MUJOCO).write(cfg.OUTPUT_XML_FILE, pretty_print=True)

  logger.info("All good, conversion procedure is finished")

  # Return path to converted file
  return cfg.OUTPUT_XML_FILE

def _parse_bodies_and_joints():
  """ Parse OpenSim `BodySet` and `JointSet`.

  :return: None
  """

  # Start from ground and go through joints recursively (depth-first), and add bodies, geoms, joints to MuJoCo model
  logger.info("Starting to parse bodies, wrapping objects, and joints")
  _add_bodies_and_joints(f"/{cfg.O_GROUND.attrib['name']}", cfg.M_WORLDBODY, root_body=True)

def _resolve_joint_frame(joint, tag_name):
  """ Resolve `socket_parent_frame` or `socket_child_frame` on an OpenSim Joint, supporting both inline `<frames>` and direct body/frame paths. """
  frame_ref = joint.find(tag_name).text.strip()
  short_name = frame_ref.split("/")[-1]
  frames = joint.find("frames")
  if frames is not None:
    local_frame = frames.find(f".//*[@name='{short_name}']")
    if local_frame is not None:
      return local_frame

  ext_frame = cfg.OPENSIM.find(f".//PhysicalOffsetFrame[@name='{short_name}']")
  if ext_frame is not None and (frame_ref == short_name or frame_ref.endswith(f"/{short_name}")):
    return ext_frame

  synth = etree.Element("PhysicalOffsetFrame", name=f"{short_name}_synth")
  etree.SubElement(synth, "socket_parent").text = frame_ref
  etree.SubElement(synth, "translation").text = "0 0 0"
  etree.SubElement(synth, "orientation").text = "0 0 0"
  return synth


def _add_bodies_and_joints(parent_name, current_body, root_body=False):
  """ Add OpenSim `Body` and related `Joint`s to MuJoCo model in a recurrent fashion.

  :param parent_name: Name of parent `Body`
  :param current_body: Pointer to current body (XML element) in the MuJoCo XML file
  :param root_body: Boolean to indicate whether current body is the root of the kinematic chain or not
  :return: None
  """
  if len(cfg.O_JOINTSET) == 0:
    return

  parent_short = parent_name.strip().split("/")[-1]

  for joint in cfg.O_JOINTSET:
    socket_parent_frame = _resolve_joint_frame(joint, "socket_parent_frame")
    socket_child_frame = _resolve_joint_frame(joint, "socket_child_frame")

    p_ref = socket_parent_frame.find("socket_parent").text.strip()
    c_ref = socket_child_frame.find("socket_parent").text.strip()

    if p_ref.split("/")[-1] != parent_short or c_ref.split("/")[-1] == parent_short:
      continue

    # Find child body of joint (supports both '/bodyset/name' paths and bare body names)
    c_tokens = [t for t in c_ref.split("/") if t]
    child_body = find_element_by_name(cfg.OPENSIM, c_tokens) if len(c_tokens) >= 2 else None
    if child_body is None:
      child_body = cfg.OPENSIM.find(f".//Body[@name='{c_tokens[-1]}']")

    # Parse body
    next_body = cfg.BODY_PARSER.parse(child_body,
                                      socket_parent_frame=socket_parent_frame,
                                      socket_child_frame=socket_child_frame,
                                      current_body=current_body,
                                      root_body=root_body)

    # Parse joint
    cfg.JOINT_PARSER.parse(joint,
                           socket_parent_frame=socket_parent_frame,
                           socket_child_frame=socket_child_frame,
                           pointer=next_body,
                           root_body=root_body)

    # Move to next joint
    _add_bodies_and_joints(c_ref, next_body)

def _parse_ground():
  """ Parse OpenSim `Ground`.

  :return: None
  """

  logger.info("Parsing the ground")
  cfg.BODY_PARSER.parse(cfg.O_GROUND, add_ground_geom=cfg.ADD_GROUND_GEOM)

def _parse_constraints():
  """ Parse OpenSim `ConstraintSet`.

  :return: None
  """

  logger.info("Starting to parse constraints")
  cfg.CONSTRAINT_PARSER.parse_all(cfg.O_CONSTRAINTSET)

def _parse_forces():
  """ Parse OpenSim `ForceSet`.

  :return: None
  """

  logger.info("Starting to parse forces, including path points and wrap paths")
  cfg.FORCE_PARSER.parse_all(cfg.O_FORCESET)

def _parse_markers():
  """ Parse OpenSim `MarkerSet`.

  :return: None
  """

  logger.info("Starting to parse markers")
  cfg.MARKER_PARSER.parse_all(cfg.O_MARKERSET)

def _set_keyframe():
  """ Create a keyframe for the MuJoCo model.

  The keyframe defines a default pose, where all dependent joints have been properly initialised.

  :return: None
  """

  logger.info("Setting the default keyframe")
  create_keyframe(cfg.MUJOCO, cfg.M_WORLDBODY, cfg.M_EQUALITY)

def _copy_credits():
  """ Copy credits from the OpenSim model.

  :return: None
  """

  # Get credits
  credits = cfg.OPENSIM.find("credits")

  # Add as comment if they exist
  if credits is not None and credits.text is not None:
    safe_credits = credits.text.replace("--", "- -").rstrip("-")
    comment = etree.Comment(f" Credits from the original OpenSim model. Note! If the following has any license information, it applies to the original OpenSim model. Credits: {safe_credits} ")
    cfg.MUJOCO.insert(0, comment)

  # Add our credits as a comment too
  comment = etree.Comment(" This model has been converted from an OpenSim model. Model conversion by MyoConverter https://github.com/MyoHub/myoconverter. This model is licensed under Apache 2.0. ")
  cfg.MUJOCO.insert(0, comment)

if __name__ == "__main__":

  argparser = argparse.ArgumentParser(description='Convert an OpenSim model into a MuJoCo model.'
                                                    'Only Works with OpenSim v4 models.')
  """ ArgumentParser: Parse arguments when running module as a Python script. """
  argparser.add_argument('xml_file', type=str,
                         help='Path to an OpenSim model XML file')
  argparser.add_argument('output_folder', type=str,
                         help="Path to an output folder. The converted model will be saved here.")
  argparser.add_argument('--geometry_folder', type=str, default=None,
                         help='Path to the Geometry folder (by default uses folder of given OpenSim file)')
  argparser.add_argument('--add_ground_geom', default=False, action="store_true",
                         help="If true, a geom (of type plane) is added to the MuJoCo model as ground")
  argparser.add_argument('--treat_as_normal_path_point', default=False, action="store_true",
                         help="If true, MovingPathPoints and ConditionalPathPoints will be treated as normal "
                              "PathPoints")
  args = argparser.parse_args()

  # Do the conversion
  convert(**vars(args))
