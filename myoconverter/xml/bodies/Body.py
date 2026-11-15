""" Contains the `Body` parser.

@author: Aleksi Ikkala
"""

from lxml import etree
from scipy.spatial.transform import Rotation

from myoconverter.xml.parsers import IParser
from myoconverter.xml import config as cfg
from myoconverter.xml.utils import str2vec, vec2str
from myoconverter.xml.bodies.utils import valid_inertia, copy_mesh_file, get_rgba


class Body(IParser):
  """ This class parses and converts the OpenSim `Body` XML element to MuJoCo. """

  def parse(self, xml, socket_parent_frame, socket_child_frame, current_body, root_body):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `Body` XML element
    :param socket_parent_frame: Socket parent frame of the `Body`
    :param socket_child_frame: Socket child frame of the `Body`
    :param current_body: Pointer to current body (XML element) in the MuJoCo XML file
    :param root_body: Boolean to indicate whether this `Body` is the root of the whole model / kinematic chain
    :return: Pointer to next body (XML element) in the MuJoCo XML file
    """

    # Get position and orientation of body
    parent_position = str2vec(socket_parent_frame.find("translation").text)
    parent_orientation = str2vec(socket_parent_frame.find("orientation").text)
    parent_rotation = Rotation.from_euler("XYZ", parent_orientation)

    # Translate/rotate body position if needed: T_{Bp, Bc} = T_{Bp, Fp} * T_{Bc, Fc}^{-1}
    child_position = str2vec(socket_child_frame.find("translation").text)
    child_orientation = str2vec(socket_child_frame.find("orientation").text)
    child_rotation = Rotation.from_euler("XYZ", child_orientation)
    rel_rotation = parent_rotation * child_rotation.inv()
    parent_position = parent_position - rel_rotation.apply(child_position) + 0.0
    parent_orientation = rel_rotation.as_euler("XYZ") + 0.0

    if root_body:
      # Does this apply to all models? Looks like it. Need to rotate by 90 degrees along x axis
      rotation = Rotation.from_euler("x", 90, degrees=True)
      parent_position = rotation.apply(parent_position)
      parent_orientation = (rotation * Rotation.from_euler("XYZ", parent_orientation)).as_euler("XYZ")

    # First you must add the body; need to define all the possible attributes somewhere
    next_body = etree.SubElement(current_body, "body",
                                 name=xml.attrib["name"],
                                 pos=vec2str(parent_position),
                                 euler=vec2str(parent_orientation))
    inertial = etree.SubElement(next_body, "inertial",
                                mass=xml.find("mass").text,
                                pos=xml.find("mass_center").text)

    # Add full inertia matrix if given mass and inertia are valid (mass is greater than zero and inertia matrix
    # eigenvalues are positive)
    # Otherwise MuJoCo will infer the inertial properties from geoms
    if valid_inertia(float(xml.find("mass").text), str2vec(xml.find("inertia").text)):
      inertial.attrib["fullinertia"] = xml.find("inertia").text

    # Add meshes (as geoms and assets)
    for mesh in xml.findall("attached_geometry/Mesh"):
      mesh_name, mesh_file = copy_mesh_file(mesh.find("mesh_file").text, cfg.GEOMETRY_FOLDER, cfg.OUTPUT_GEOMETRY_FOLDER)
      etree.SubElement(cfg.M_ASSET, "mesh",
                       name=f"{mesh.attrib['name']}_{mesh_name}",
                       file=f"Geometry/{mesh_file}",
                       scale=mesh.find("scale_factors").text)
      etree.SubElement(next_body, "geom",
                       name=mesh.attrib["name"],
                       type="mesh",
                       mesh=f"{mesh.attrib['name']}_{mesh_name}",
                       rgba=get_rgba(mesh),
                       group="0")

      # Add also contacts between geoms and ground (if ground geom exists)
      if cfg.ADD_GROUND_GEOM:
        etree.SubElement(cfg.M_CONTACT, "pair",
                         geom1="ground-plane",
                         geom2=mesh.attrib["name"])

    # Parse wrapping objects, if there are any
    cfg.WRAP_OBJECT_PARSER.parse_all(xml.find("WrapObjectSet/objects"), m_body=next_body)

    return next_body
