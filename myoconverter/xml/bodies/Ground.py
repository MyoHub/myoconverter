""" Contains the Ground parser.

@author: Aleksi Ikkala
"""

from scipy.spatial.transform import Rotation
from lxml import etree
import os

from myoconverter.xml.parsers import IParser
from myoconverter.xml import config as cfg
from myoconverter.xml.utils import vec2str
from myoconverter.xml.bodies.utils import get_rgba, copy_mesh_file


class Ground(IParser):
  """ This class parses and converts the OpenSim `Ground` XML element to MuJoCo. """

  def parse(self, xml, add_ground_geom=False):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Ground` XML element
    :param add_ground_geom: Boolean to indicate whether a "ground" plane geom should be added to the MuJoCo model
    :return: None
    """

    # Set name of ground body in MuJoCo
    cfg.M_GROUND.attrib["name"] = xml.attrib["name"]

    # We need to rotate the body by 90 degrees wrt x axis
    rotation = Rotation.from_euler("x", 90, degrees=True)
    cfg.M_GROUND.attrib["euler"] = vec2str(rotation.as_euler("XYZ"))

    # Add a ground geom if so instructed
    if add_ground_geom:
      etree.SubElement(cfg.M_GROUND, "geom",
                       name="ground-plane",
                       pos="0 0 0",
                       euler=vec2str(rotation.inv().as_euler("XYZ")),
                       size="10 10 0.125",
                       type="plane",
                       rgba="1.0 0.7 0.4 1.0",
                       condim="3")

    # Check if there are one or multiple components
    if xml.find("components") is not None:
      for component in xml.find("components"):
        self._parse_component(component)
    else:
      self._parse_component(xml)

    # Parse wrapping objects, if there are any
    cfg.WRAP_OBJECT_PARSER.parse_all(xml.find("WrapObjectSet/objects"), m_body=cfg.M_GROUND)

  def _parse_component(self, xml):

    # Check if these need to be translated or rotated
    translation = xml.find("translation")
    pos = "0 0 0" if translation is None else translation.text
    orientation = xml.find("orientation")
    euler = "0 0 0" if orientation is None else orientation.text

    # Copy any attached geometries
    for mesh in xml.findall("attached_geometry/Mesh"):
      mesh_name, mesh_file = copy_mesh_file(mesh.find("mesh_file").text, cfg.GEOMETRY_FOLDER, cfg.OUTPUT_GEOMETRY_FOLDER)
      etree.SubElement(cfg.M_ASSET, "mesh",
                       name=f"{mesh.attrib['name']}_{mesh_name}",
                       file=os.path.join("Geometry", mesh_file),
                       scale=mesh.find("scale_factors").text)
      etree.SubElement(cfg.M_GROUND, "geom",
                       name=mesh.attrib["name"],
                       type="mesh",
                       pos=pos,
                       euler=euler,
                       mesh=f"{mesh.attrib['name']}_{mesh_name}",
                       rgba=get_rgba(mesh),
                       group="0")