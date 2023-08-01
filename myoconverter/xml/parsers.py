""" This module defines parsers (classes) that collect individual parsers from their respective packages.

The individual parsers and respective packages are defined under myoconverter.xml
"""

import os
import importlib
from abc import ABC, abstractmethod

from loguru import logger

from myoconverter.xml.utils import filter_set
from myoconverter.xml import config as cfg


class BaseParser:
  """ Defines a base class for parsers.

  The individual parsers derived from this class handle element specific parsing and converting. In addition to
  inheriting from this class, all parsers must implement the IParser interface.
  """

  def __init__(self, component, ignore_files=set()):
    self.component = component

    # Get files in component package
    folder = os.path.dirname(os.path.realpath(__file__))
    files = set(os.listdir(os.path.join(folder, component))) - ignore_files

    # Collect all parsers
    self.parsers = self._collect_parsers(files)

  def parse(self, xml, **kwargs):
    """ Calls the correct parser for given XML element.

    :param xml: OpenSim XML element to be parsed and converted to MuJoCo model
    :param kwargs: Optional keyword arguments for parsers
    :return: Optional return value defined by the parser that is called
    :raises: NotImplementedError: If a parser for given XML has not been implemented
    :raises: RuntimeWarning: If a parser has been implemented, but doesn't follow the IParser interface
    """

    if xml is None:
      return

    # Check if a parser has been implemented
    if xml.tag not in self.parsers:
      logger.critical(f"Parser for {self.component} type {xml.tag} has not been implemented. "
                      f"See {cfg.URL_API_XML} for a list of implemented parsers.")
      raise NotImplementedError

    # Check that the parser implements the interface
    if not isinstance(self.parsers[xml.tag], IParser):
      logger.error(f"Parser {self.parsers[xml.tag]} does not implement the {IParser} interface, "
                   f"something might break")
      raise RuntimeWarning

    # Parse -- some parsers may return values
    logger.info(f"[{self.__class__.__name__}: {xml.tag}] {xml.attrib['name'] if 'name' in xml.attrib else ''}")
    return self.parsers[xml.tag].parse(xml, **kwargs)

  def parse_all(self, objects, **kwargs):
    """ A utility function to parse all elements in given list of objects

    :param objects: A list of XML elements
    :param kwargs: Optional keyword arguments
    :return: None
    """
    if objects is not None:
      for xml in objects:
        self.parse(xml, **kwargs)

  def _collect_parsers(self, files):
    """ Collect all implemented parsers

    :param files: A list of files, each defines a parser
    :return: A dictionary with all parsers defined in given files
    """
    parsers = dict()

    # Go through all files
    # Files starting with "__" will be ignored
    for file in filter_set(files, "__"):

      # Get name of parser
      name = file[:-3]

      # Import module
      module = importlib.import_module(f"myoconverter.xml.{self.component}.{name}")

      # Initialise the object
      parsers[name] = getattr(module, name)()

    return parsers

class BodyParser(BaseParser):
  """ This class collects all parsers relevant to parsing bodies.

    All body parsers should be inside :py:mod:`myoconverter.xml.bodies` package. One class per file, with class name
    matching file name.
  """
  def __init__(self):
    super().__init__("bodies", ignore_files={"utils.py"})

class ConstraintParser(BaseParser):
  """ This class collects all parsers relevant to parsing constraints.

    All constraint parsers should be inside :py:mod:`myoconverter.xml.constraints` package. One class per file, with
    class name matching file name.
  """
  def __init__(self):
    super().__init__("constraints", ignore_files=set())

class JointParser(BaseParser):
  """ This class collects all parsers relevant to parsing joints.

    All joint parsers should be inside :py:mod:`myoconverter.xml.joints` package. One class per file, with class name
    matching file name.
  """
  def __init__(self):
    super().__init__("joints", ignore_files={"utils.py", "Joint.py"})

class ForceParser(BaseParser):
  """ This class collects all parsers relevant to parsing forces.

    All force parsers should be inside :py:mod:`myoconverter.xml.forces` package. One class per file, with class name
    matching file name.
  """
  def __init__(self):
    super().__init__("forces", ignore_files={"utils.py", "Muscle.py"})

class PathPointParser(BaseParser):
  """ This class collects all parsers relevant to parsing path points.

    All path point parsers should be inside :py:mod:`myoconverter.xml.path_points` package. One class per file, with
    class name matching file name.
  """

  def __init__(self):
    super().__init__("path_points", ignore_files={"utils.py"})

class WrapObjectParser(BaseParser):
  """ This class collects all parsers relevant to parsing wrap objects.

    All wrapping object parsers should be inside :py:mod:`myoconverter.xml.wrap_objects` package. One class per file,
    with class name matching file name.
  """
  def __init__(self):
    super().__init__("wrap_objects", ignore_files={"utils.py", "WrapObject.py"})

class PathWrapParser(BaseParser):
  """ This class collects all parsers relevant to parsing path wraps (except see PathWrapSetParser).

    All path wrap parsers should be inside :py:mod:`myoconverter.xml.path_wraps` package. One class per file, with class
    name matching file name.
  """
  def __init__(self):
    super().__init__("path_wraps", ignore_files={"utils.py", "PathWrapSet_OLD.py", "PathWrapSet.py"})

class PathWrapSetParser(BaseParser):
  """ A higher hierarchy level parser for parsing PathWrapSets, where we need to keep track of multiple path wraps
  during parsing. """
  def __init__(self):
    super().__init__("path_wraps", ignore_files={"utils.py", "PathWrap.py"})

class MarkerParser(BaseParser):
  """ This class collects all parsers relevant to parsing markers.

    All marker parsers should be inside :py:mod:`myoconverter.xml.markers` package. One class per file, with class name
    matching file name.
  """
  def __init__(self):
    super().__init__("markers", ignore_files=set())


class IParser(ABC):
  """ This class defines an interface that the individual parsers must implement. """

  @abstractmethod
  def parse(self, **kwargs):
    pass