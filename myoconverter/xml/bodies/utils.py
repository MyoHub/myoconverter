""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `BodySet`

@author: Aleksi Ikkala
"""

import numpy as np
import os
import trimesh
import pyvista
from shutil import copyfile

from loguru import logger

from myoconverter.xml.utils import create_symmetric_matrix


def valid_inertia(mass, inertia_vec):
  """ Check if given inertia vector is valid and can be used in the MuJoCo model.

  :param mass: Mass of a body/geom
  :param inertia_vec: Inertia vector
  :return: Boolean indicating whether the given vector is valid
  """
  values, vectors = np.linalg.eig(create_symmetric_matrix(inertia_vec))
  if mass > 0 and np.all(values > 0):
    return True
  else:
    return False

def copy_mesh_file(mesh_file, geometry_folder, output_geometry_folder):
  """ This function copies the original mesh file,converts it to stl and fixes unconnected facets (if necessary)

  :param mesh_file: Path to mesh file
  :param geometry_folder: Path to the folder where the mesh file is
  :param output_geometry_folder: Path to folder into which the mesh file will be copied to
  :return: Name of mesh, filename of converted stl mesh file
  :raises NotImplementedError: if given mesh file is not vtk or stl
  """
  mesh_name = mesh_file[:-4]
  stl_filename = mesh_name + ".stl"
  stl_filepath = os.path.join(output_geometry_folder, stl_filename)

  # Transform a vtk file into an stl file and repair the mesh
  if mesh_file[-3:] == "vtp":

    # Read the vtp file with pyvista
    pymesh = pyvista.read(os.path.join(geometry_folder, mesh_file))

    # Convert into trimesh (see https://github.com/pyvista/pyvista/discussions/2268)
    pymesh = pymesh.extract_surface().triangulate()
    faces_as_array = pymesh.faces.reshape((pymesh.n_faces, 4))[:, 1:]
    tmesh = trimesh.Trimesh(pymesh.points, faces_as_array)

    # Repair the mesh
    trimesh.repair.fix_inversion(tmesh, True)
    trimesh.repair.fix_normals(tmesh, True)
    trimesh.repair.fill_holes(tmesh)

    # Save as binary stl file
    tmesh.export(stl_filepath)

  # Just copy the stl file
  elif mesh_file[-3:] == "stl":
    copyfile(os.path.join(geometry_folder, mesh_file), stl_filepath)

  else:
    logger.critical("Geom file is not vtp or stl!")
    raise NotImplementedError

  return mesh_name, stl_filename

def get_rgba(mesh):
  """ Return rgba string for given mesh

  :param mesh: XML element
  :return: Rgba string in format "r g b a"
  """
  color = mesh.find("Appearance/color")
  opacity = mesh.find("Appearance/opacity")
  visible = mesh.find("Appearance/visible")

  # Check if mesh is visible, or transparent
  if visible is not None:
    if visible.text=="false":
      alpha = "0"
    else:
      alpha = "1"
  elif opacity is None:
    alpha = "1"
  else:
    alpha = opacity.text

  if alpha == "0":
    logger.warning(f"Setting mesh {mesh.attrib['name']} to invisible")

  return ("1 1 1" if color is None else color.text) + " " + alpha
