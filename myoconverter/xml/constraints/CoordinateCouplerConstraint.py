""" Contains the `CoordinateCouplerConstraint` parser.

@author: Aleksi Ikkala
"""

from lxml import etree
import numpy as np
import matplotlib
import matplotlib.pyplot as pp
import os

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2vec, vec2str, fit_spline
from myoconverter.xml import config as cfg

matplotlib.use("agg")
pp.ioff()


class CoordinateCouplerConstraint(IParser):
  """ This class parses and converts the OpenSim `CoordinateCouplerConstraint` XML element to MuJoCo. """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `CoordinateCouplerConstraint` XML element
    :return: None
    :raises: NotImplementedError: If function type is other than 'SimmSpline', 'NaturalCubicSpline', or 'LinearFunction'
    :raises: RuntimeError: If multiple independent coordinates must implement this constraint
    """

    # Only add the dependency if dependent coordinate exists (has been designated as a dof). Sometimes the dependent
    # coordinate is not independent, but rather already has other equality constraints (like knee_angle_pat_r in
    # Leg6Dof9Musc)
    dependent_coordinate_name = xml.find("dependent_coordinate_name").text

    # Get independent coordinate
    independent_coordinate_names = xml.find("independent_coordinate_names").text

    # Make sure there's only one independent coordinate. Not sure how they would be separated since I haven't ever seen
    # a case with multiple independent coordinates. Try splitting with a space or a comma
    if len(independent_coordinate_names.split(" ")) > 2 or len(independent_coordinate_names.split(",")) > 2:
      logger.critical(f"Multiple independent coordinates not supported (CoordinateCouplerConstraint "
                       f"{xml.attrib['name']})")
      raise RuntimeError

    # Get the dependency
    coupled_coordinates_function = xml.find("coupled_coordinates_function").getchildren()[0]
    if coupled_coordinates_function.tag in ["SimmSpline", "NaturalCubicSpline"]:

      # Get x and y values that define the spline
      x_values = str2vec(coupled_coordinates_function.find(".//x").text)
      y_values = str2vec(coupled_coordinates_function.find(".//y").text)

      # Fit a spline
      fit, polycoef, _ = fit_spline(x_values, y_values)

      # Do some plotting; check if output folder exists
      output_dir = os.path.join(cfg.OUTPUT_PLOT_FOLDER, "coordinate_coupler_constraints")
      if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

      # Plot and save figure
      fig = self._plot_figure(x_values, y_values, fit, independent_coordinate_names, dependent_coordinate_name,
                              xml.attrib["name"])
      pp.savefig(os.path.join(output_dir, f"{xml.attrib['name']}.svg"))
      pp.close(fig)

    elif coupled_coordinates_function.tag == "LinearFunction":

      # Get coefficients of the linear function
      coefs = str2vec(coupled_coordinates_function.find("coefficients").text)

      # Make a quartic representation of the linear function
      polycoef = np.zeros((5,))
      polycoef[0] = coefs[1]
      polycoef[1] = coefs[0]

      # Dummy linear fit function
      fit = np.polynomial.polynomial.Polynomial.fit([0, 1], [0, 1], 1)

    else:
      logger.critical(f"Function type {coupled_coordinates_function.tag} has not been implemented")
      raise NotImplementedError

    # Create a constraint
    etree.SubElement(cfg.M_EQUALITY, "joint",
                     name=xml.attrib["name"],
                     joint1=dependent_coordinate_name,
                     joint2=independent_coordinate_names,
                     active=xml.find("isEnforced").text,
                     polycoef=vec2str(polycoef),
                     solimp="0.9999 0.9999 0.001 0.5 2")

  def _plot_figure(self, x_values, y_values, fit, independent_coordinate, dependent_coordinate, constraint_name):

    # Initialise figure
    fig = pp.figure(figsize=(10, 8))

    # Calculate the quartic approximation
    x_approx = np.linspace(min(x_values), max(x_values), 100)
    y_approx = fit(x_approx)

    # Plot the OpenSim data points and approximation
    pp.plot(x_values, y_values, '.', markersize=10, label=f"OpenSim constraint function data points")
    pp.plot(x_approx, y_approx, label=f"Approximation of the constraint function")
    pp.legend()
    pp.xlabel(f"Independent joint value ({independent_coordinate})")
    pp.ylabel(f"Dependent joint value ({dependent_coordinate})")
    pp.title(f"Approximation of CoordinateCouplerConstraint {constraint_name}")

    return fig
