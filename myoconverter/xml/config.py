""" Contains definitions for global variables that are used in the conversion process.

@author: Aleksi Ikkala
"""
import os
from lxml import etree

from loguru import logger

from myoconverter.xml.parsers import BodyParser, ConstraintParser, ForceParser, JointParser, PathPointParser, \
  PathWrapParser, PathWrapSetParser, WrapObjectParser, MarkerParser


# Variables for paths, model names
OUTPUT_FOLDER = None
GEOMETRY_FOLDER = None
OUTPUT_GEOMETRY_FOLDER = None
MODEL_NAME = None
OUTPUT_XML_FILE = None
OUTPUT_LOG_FILE = None
OUTPUT_PLOT_FOLDER = None

# Parsing parameters
ADD_GROUND_GEOM = None
TREAT_AS_NORMAL_PATH_POINT = None

# Variables for parsed OpenSim XML
OPENSIM = None
O_GROUND = None
O_BODYSET = None
O_JOINTSET = None
O_FORCESET = None
O_CONSTRAINTSET = None

# Variables for parsed MuJoCo XML
MUJOCO = None
M_WORLDBODY = None
M_GROUND = None
M_ASSET = None
M_TENDON = None
M_ACTUATOR = None
M_EQUALITY = None
M_CONTACT = None

# Variables for parsers
BODY_PARSER = None
CONSTRAINT_PARSER = None
FORCE_PARSER = None
JOINT_PARSER = None
PATH_POINT_PARSER = None
PATH_WRAP_PARSER = None
PATH_WRAP_SET_PARSER = None
WRAP_OBJECT_PARSER = None
MARKER_PARSER = None

# Some URLs
URL_API = None
URL_API_XML = None
URL_HOME = None


def initialise(xml_file, output_folder, **kwargs):
  """ Initialise the configs / global variables.

  :param xml_file: The OpenSim XML file
  :param output_folder: Output folder where the MuJoCo model will be saved into
  :param kwargs: Optional keyword arguments
  :return: None
  """

  # Set folder paths
  global OUTPUT_FOLDER, GEOMETRY_FOLDER, OUTPUT_GEOMETRY_FOLDER, MODEL_NAME, OUTPUT_XML_FILE, OUTPUT_LOG_FILE, \
    OUTPUT_PLOT_FOLDER
  OUTPUT_FOLDER = output_folder
  OUTPUT_GEOMETRY_FOLDER = os.path.join(OUTPUT_FOLDER, "Geometry")
  MODEL_NAME = os.path.split(xml_file)[1][:-5]
  OUTPUT_XML_FILE = os.path.join(OUTPUT_FOLDER, MODEL_NAME + "_cvt1.xml")
  # OUTPUT_LOG_FILE = os.path.join(OUTPUT_FOLDER, f"{MODEL_NAME}-conversion.log")
  OUTPUT_PLOT_FOLDER = os.path.join(OUTPUT_FOLDER, "Step1_xmlConvert")

  # Set URLs
  global URL_API, URL_API_XML, URL_HOME
  URL_HOME = "https://myoconverter.readthedocs.io/en/latest/"
  URL_API = "https://myoconverter.readthedocs.io/en/latest/autoapi/index.html"
  URL_API_XML = "https://myoconverter.readthedocs.io/en/latest/autoapi/myoconverter/xml/index.html"

  if "geometry_folder" not in kwargs or kwargs["geometry_folder"] is None:
    GEOMETRY_FOLDER = os.path.join(os.path.dirname(xml_file), "Geometry")
  else:
    GEOMETRY_FOLDER = kwargs["geometry_folder"]

  # Create the output and geometry folders
  os.makedirs(OUTPUT_FOLDER, exist_ok=True)
  os.makedirs(OUTPUT_GEOMETRY_FOLDER, exist_ok=True)

  # Create an XML parser
  parser = etree.XMLParser(remove_blank_text=True)

  # Set parsing parameters
  global ADD_GROUND_GEOM
  ADD_GROUND_GEOM = kwargs.get("add_ground_geom", False)

  # Optional parameters for users
  global TREAT_AS_NORMAL_PATH_POINT
  TREAT_AS_NORMAL_PATH_POINT = kwargs.get("treat_as_normal_path_point", False)

  # Read and parse OpenSim model
  global OPENSIM, O_GROUND, O_BODYSET, O_JOINTSET, O_FORCESET, O_CONSTRAINTSET, O_MARKERSET
  OPENSIM = etree.parse(xml_file, parser).getroot().find("Model")
  O_GROUND = OPENSIM.find("Ground")
  O_BODYSET = OPENSIM.find("BodySet/objects")
  O_BODYSET = [] if O_BODYSET is None else O_BODYSET
  O_JOINTSET = OPENSIM.find("JointSet/objects")
  O_JOINTSET = [] if O_JOINTSET is None else O_JOINTSET
  O_FORCESET = OPENSIM.find("ForceSet/objects")
  O_FORCESET = [] if O_FORCESET is None else O_FORCESET
  O_CONSTRAINTSET = OPENSIM.find("ConstraintSet/objects")
  O_CONSTRAINTSET = [] if O_CONSTRAINTSET is None else O_CONSTRAINTSET
  O_MARKERSET = OPENSIM.find("MarkerSet/objects")
  O_MARKERSET = [] if O_MARKERSET is None else O_MARKERSET

  # Read and parse MuJoCo template
  global MUJOCO, M_GROUND, M_WORLDBODY, M_ASSET, M_TENDON, M_ACTUATOR, M_EQUALITY, M_CONTACT
  MUJOCO = etree.parse(os.path.join(os.path.dirname(os.path.realpath(__file__)), "template.xml"), parser).getroot()
  M_WORLDBODY = MUJOCO.find("worldbody")
  M_GROUND = M_WORLDBODY.find("body[@name='ground']")
  M_ASSET = MUJOCO.find("asset")
  M_TENDON = MUJOCO.find("tendon")
  M_ACTUATOR = MUJOCO.find("actuator")
  M_EQUALITY = MUJOCO.find("equality")
  M_CONTACT = MUJOCO.find("contact")

  # Create all parsers
  # NOTE: Initialisation order matters here, because e.g BodyParser imports WRAP_OBJECT_PARSER, so it must be defined
  # first, and PathWrapSetParser imports PATH_WRAP_PARSER, so it must be defined first
  global BODY_PARSER, CONSTRAINT_PARSER, FORCE_PARSER, JOINT_PARSER, PATH_POINT_PARSER, PATH_WRAP_PARSER, \
    PATH_WRAP_SET_PARSER, WRAP_OBJECT_PARSER, MARKER_PARSER
  WRAP_OBJECT_PARSER = WrapObjectParser()
  PATH_WRAP_PARSER = PathWrapParser()
  PATH_WRAP_SET_PARSER = PathWrapSetParser()
  PATH_POINT_PARSER = PathPointParser()
  BODY_PARSER = BodyParser()
  FORCE_PARSER = ForceParser()
  JOINT_PARSER = JointParser()
  CONSTRAINT_PARSER = ConstraintParser()
  MARKER_PARSER = MarkerParser()
