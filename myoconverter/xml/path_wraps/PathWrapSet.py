""" Contains the `PathWrapSet` parser.

@author: Aleksi Ikkala
"""

from lxml import etree
from copy import deepcopy

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml import config as cfg


class PathWrapSet(IParser):
  """ This class parses and converts the OpenSim `PathWrapSet` XML element to MuJoCo.

  This class parses a whole PathWrapSet, unlike other parsers that parse individual objects of sets. This is because
  a single tendon may have multiple path wraps (over different wrapping objects), but we can wrap the MuJoCo tendon only
  wrt one wrapping object at a time. Hence, we wrap the MuJoCo tendon over the wrapping object that is closest.

  In this parser the estimation of wrapping sites is very much based on heuristics. We calculate distances between
  tendons (or segments of tendons) and all applicable wrapping objects, and decide the wrapping sites based on those
  distances -- if a segment (two consecutive sites) is close enough to a wrapping object, we add a wrapping site between
  those sites. Works only with stationary sites. Also, relies on the assumption that wrapping objects are always close
  to the tendons (distances are estimated when MuJoCo model is in default pose).
  """

  def parse(self, xml, tendon, force_name):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `PathWrapSet` XML element
    :param tendon: MuJoCo `tendon/spatial` XML element
    :param force_name: Name of an actuator/tendon that wraps around this object
    :return: None
    """

    if xml.find("objects") is None:
      return

    # Go through all path wraps
    params = dict()
    issue_warning = False
    for path_wrap in xml.find("objects"):
      warn = cfg.PATH_WRAP_PARSER.parse(path_wrap, tendon=tendon, params=params)
      issue_warning = issue_warning or warn

    if issue_warning:
      logger.warning(f"One or multiple sites in tendon {tendon.attrib['name']} are dynamically moving, and hence there "
                     f"may be missing wrapping sites. You should check the converted xml file yourself, and possibly "
                     f"add wrapping sites where needed (or modify the ranges of corresponding PathWraps in the OpenSim "
                     f"model). ")

    # Transform params dict into a list
    params_list = [(k,v) for k, v in params.items()]

    # Add wrapping sites (in reverse order). sorted sorts according to first item in each tuple by default
    for p in sorted(params_list, reverse=True):

      # Create the sidesite
      p[1]["wrap_object_body"].append(p[1]["sidesite"])

      # When dealing with ellipsoid wrapping objects, we will create unique sphere wrapping objects for each muscle-
      # wrapping object pair, so we can optimize their locations separately for each muscle. We'll create the wrapping
      # objects here on demand, as opposed to automatically generating them for each muscle-wrapping object pair

      # Check if this wrapping object is an ellipsoid
      split = p[1]["wrap_object"].split("_")
      if split[-1] == "ellipsoid":
        unique_name = p[1]["wrap_object"] + "_" + force_name

        # Check if a wrapping object has already been created for this muscle
        if cfg.M_WORLDBODY.find(f".//geom[@name='{unique_name}']") is None:

          # Get the original wrapping object
          wrap_object = cfg.M_WORLDBODY.find(f".//geom[@name='{p[1]['wrap_object']}']")

          # Copy the attributes, update name
          attrib = deepcopy(wrap_object.attrib)
          attrib["name"] = unique_name

          # Create the new wrapping object
          etree.SubElement(wrap_object.getparent(), wrap_object.tag, attrib)

        # Switch reference to the unique wrapping object
        p[1]["wrap_object"] = unique_name

      # Add the wrapping site to tendon
      tendon.insert(p[0], etree.Element("geom", geom=p[1]["wrap_object"], sidesite=p[1]["sidesite"].attrib["name"]))