"""Tests for OpenSim ``CoordinateLimitForce`` conversion to MuJoCo joint-limit impedance."""

from __future__ import annotations

import math
import sys
import types

import mujoco
import numpy as np
import pytest
from lxml import etree

try:
    import pyvista as _pv  # noqa: F401
except ModuleNotFoundError:
    sys.modules["pyvista"] = types.ModuleType("pyvista")

from myoconverter.xml.converter import convert


def _build_osim_v4_xml(*, bodies_xml: str, joints_xml: str, forces_xml: str = "") -> str:
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
  <Model name="clf_test_model">
    <credits>CoordinateLimitForce Test Model</credits>
    <publications>None</publications>
    <length_units>meters</length_units>
    <force_units>N</force_units>
    <gravity>0 -9.80665 0</gravity>
    <Ground name="ground">
      <WrapObjectSet name="wrapobjectset"><objects /><groups /></WrapObjectSet>
    </Ground>
    <BodySet name="bodyset">
      <objects>
        {bodies_xml}
      </objects>
      <groups />
    </BodySet>
    <JointSet name="jointset">
      <objects>
        {joints_xml}
      </objects>
      <groups />
    </JointSet>
    <ConstraintSet name="constraintset"><objects /><groups /></ConstraintSet>
    <ForceSet name="forceset">
      <objects>
        {forces_xml}
      </objects>
      <groups />
    </ForceSet>
    <MarkerSet name="markerset"><objects /><groups /></MarkerSet>
  </Model>
</OpenSimDocument>
"""


def test_coordinate_limit_force_rotational_hinge_and_passive_limit_physics(tmp_path):
    """Verify deg->rad limits, N*m/deg->N*m/rad impedance, flags, and restoring torque in MuJoCo."""
    bodies_xml = """
    <Body name="humerus">
      <mass>1.5</mass>
      <mass_center>0 -0.15 0</mass_center>
      <inertia>0.012 0.002 0.012 0 0 0</inertia>
      <WrapObjectSet name="wrapobjectset"><objects /><groups /></WrapObjectSet>
    </Body>
    <Body name="forearm">
      <mass>1.2</mass>
      <mass_center>0 -0.12 0</mass_center>
      <inertia>0.008 0.001 0.008 0 0 0</inertia>
      <WrapObjectSet name="wrapobjectset"><objects /><groups /></WrapObjectSet>
    </Body>
    """
    joints_xml = """
    <PinJoint name="shoulder">
      <socket_parent_frame>ground_offset</socket_parent_frame>
      <socket_child_frame>humerus_offset</socket_child_frame>
      <coordinates>
        <Coordinate name="shoulder_elv">
          <default_value>0</default_value>
          <range>-1.5708 1.5708</range>
          <clamped>false</clamped>
          <locked>false</locked>
        </Coordinate>
      </coordinates>
      <frames>
        <PhysicalOffsetFrame name="ground_offset">
          <socket_parent>/ground</socket_parent>
          <translation>0 0.8 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
        <PhysicalOffsetFrame name="humerus_offset">
          <socket_parent>/bodyset/humerus</socket_parent>
          <translation>0 0 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
      </frames>
    </PinJoint>
    <PinJoint name="elbow">
      <socket_parent_frame>humerus_distal</socket_parent_frame>
      <socket_child_frame>forearm_prox</socket_child_frame>
      <coordinates>
        <Coordinate name="elbow_flex">
          <default_value>0.5</default_value>
          <range>0.0 2.5</range>
          <clamped>true</clamped>
          <locked>false</locked>
        </Coordinate>
      </coordinates>
      <frames>
        <PhysicalOffsetFrame name="humerus_distal">
          <socket_parent>/bodyset/humerus</socket_parent>
          <translation>0 -0.3 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
        <PhysicalOffsetFrame name="forearm_prox">
          <socket_parent>/bodyset/forearm</socket_parent>
          <translation>0 0 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
      </frames>
    </PinJoint>
    """
    forces_xml = """
    <CoordinateLimitForce name="elbow_limit_stop">
      <appliesForce>true</appliesForce>
      <isDisabled>false</isDisabled>
      <coordinate>elbow_flex</coordinate>
      <upper_limit>90.0</upper_limit>
      <upper_stiffness>2.0</upper_stiffness>
      <lower_limit>15.0</lower_limit>
      <lower_stiffness>2.0</lower_stiffness>
      <damping>0.05</damping>
      <transition>5.0</transition>
    </CoordinateLimitForce>
    <CoordinateLimitForce name="shoulder_soft_stop_from_unclamped">
      <appliesForce>true</appliesForce>
      <coordinate>shoulder_elv</coordinate>
      <upper_limit>60.0</upper_limit>
      <upper_stiffness>1.5</upper_stiffness>
      <lower_limit>-30.0</lower_limit>
      <lower_stiffness>1.5</lower_stiffness>
      <damping>0.02</damping>
      <transition>3.0</transition>
    </CoordinateLimitForce>
    <CoordinateLimitForce name="disabled_stop_flag">
      <isDisabled>true</isDisabled>
      <coordinate>elbow_flex</coordinate>
      <upper_limit>30.0</upper_limit>
      <upper_stiffness>999.0</upper_stiffness>
      <lower_limit>20.0</lower_limit>
      <lower_stiffness>999.0</lower_stiffness>
      <damping>50.0</damping>
      <transition>1.0</transition>
    </CoordinateLimitForce>
    <CoordinateLimitForce name="applies_force_false_stop">
      <appliesForce>false</appliesForce>
      <coordinate>elbow_flex</coordinate>
      <upper_limit>30.0</upper_limit>
      <upper_stiffness>999.0</upper_stiffness>
      <lower_limit>20.0</lower_limit>
      <lower_stiffness>999.0</lower_stiffness>
      <damping>50.0</damping>
      <transition>1.0</transition>
    </CoordinateLimitForce>
    <CoordinateLimitForce name="nonexistent_coord_stop">
      <appliesForce>true</appliesForce>
      <coordinate>nonexistent_dof</coordinate>
      <upper_limit>45.0</upper_limit>
      <upper_stiffness>5.0</upper_stiffness>
      <lower_limit>-45.0</lower_limit>
      <lower_stiffness>5.0</lower_stiffness>
      <damping>0.1</damping>
      <transition>2.0</transition>
    </CoordinateLimitForce>
    """

    osim_path = tmp_path / "arm_clf.osim"
    osim_path.write_text(
        _build_osim_v4_xml(
            bodies_xml=bodies_xml, joints_xml=joints_xml, forces_xml=forces_xml
        ),
        encoding="utf-8",
    )

    out_xml = convert(str(osim_path), str(tmp_path / "out"))
    tree = etree.parse(out_xml)

    elbow_jnt = tree.find(".//joint[@name='elbow_flex']")
    assert elbow_jnt is not None
    assert elbow_jnt.attrib["limited"] == "true"

    elbow_range = [float(v) for v in elbow_jnt.attrib["range"].split()]
    np.testing.assert_allclose(
        elbow_range, [math.radians(15.0), math.radians(90.0)], rtol=1e-4
    )

    # Damping is additive with template default (0.05 + 0.05 * 180 / pi)
    expected_damping = 0.05 + 0.05 * (180.0 / math.pi)
    assert float(elbow_jnt.attrib["damping"]) == pytest.approx(
        expected_damping, rel=1e-4
    )

    solimp = [float(v) for v in elbow_jnt.attrib["solimplimit"].split()]
    np.testing.assert_allclose(
        solimp,
        [0.0001, 0.99, math.radians(5.0), 0.5, 1.0],
        rtol=1e-4,
    )

    solref = [float(v) for v in elbow_jnt.attrib["solreflimit"].split()]
    expected_k = 2.0 * (180.0 / math.pi)
    expected_b = 0.05 * (180.0 / math.pi)
    np.testing.assert_allclose(solref, [-expected_k, -expected_b], rtol=1e-4)

    # Unclamped root joint becomes limited="true" with [-30, 60] deg in radians
    shoulder_jnt = tree.find(".//joint[@name='shoulder_elv']")
    assert shoulder_jnt.attrib["limited"] == "true"
    shoulder_range = [float(v) for v in shoulder_jnt.attrib["range"].split()]
    np.testing.assert_allclose(
        shoulder_range, [math.radians(-30.0), math.radians(60.0)], rtol=1e-4
    )

    # Compile in MuJoCo and verify passive limit torques engage outside [15 deg, 90 deg]
    model = mujoco.MjModel.from_xml_path(out_xml)
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    elbow_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow_flex")
    qadr = int(model.jnt_qposadr[elbow_jid])
    dadr = int(model.jnt_dofadr[elbow_jid])

    data.qpos[qadr] = math.radians(45.0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    assert data.qfrc_constraint[dadr] == pytest.approx(0.0, abs=1e-9)

    data.qpos[qadr] = math.radians(105.0)
    mujoco.mj_forward(model, data)
    tau_105 = float(data.qfrc_constraint[dadr])
    assert tau_105 < -0.5

    data.qpos[qadr] = math.radians(120.0)
    mujoco.mj_forward(model, data)
    assert float(data.qfrc_constraint[dadr]) < tau_105

    data.qpos[qadr] = math.radians(-5.0)
    mujoco.mj_forward(model, data)
    tau_minus5 = float(data.qfrc_constraint[dadr])
    assert tau_minus5 > 0.2

    data.qpos[qadr] = math.radians(-20.0)
    mujoco.mj_forward(model, data)
    assert float(data.qfrc_constraint[dadr]) > tau_minus5


def test_coordinate_limit_force_translational_slide_joint(tmp_path):
    """Verify translational ``slide`` joints keep meter / N/m units without deg/rad scaling."""
    bodies_xml = """
    <Body name="carriage">
      <mass>2.0</mass>
      <mass_center>0 0 0</mass_center>
      <inertia>0.02 0.02 0.02 0 0 0</inertia>
      <WrapObjectSet name="wrapobjectset"><objects /><groups /></WrapObjectSet>
    </Body>
    """
    joints_xml = """
    <SliderJoint name="rail">
      <socket_parent_frame>ground_offset</socket_parent_frame>
      <socket_child_frame>carriage_offset</socket_child_frame>
      <coordinates>
        <Coordinate name="rail_tx">
          <default_value>0</default_value>
          <range>-1.0 1.0</range>
          <clamped>false</clamped>
          <locked>false</locked>
        </Coordinate>
      </coordinates>
      <frames>
        <PhysicalOffsetFrame name="ground_offset">
          <socket_parent>/ground</socket_parent>
          <translation>0 0 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
        <PhysicalOffsetFrame name="carriage_offset">
          <socket_parent>/bodyset/carriage</socket_parent>
          <translation>0 0 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
      </frames>
    </SliderJoint>
    """
    forces_xml = """
    <CoordinateLimitForce name="rail_stop">
      <appliesForce>true</appliesForce>
      <coordinate>rail_tx</coordinate>
      <upper_limit>0.12</upper_limit>
      <upper_stiffness>400.0</upper_stiffness>
      <lower_limit>-0.05</lower_limit>
      <lower_stiffness>400.0</lower_stiffness>
      <damping>15.0</damping>
      <transition>0.008</transition>
    </CoordinateLimitForce>
    """
    osim_path = tmp_path / "slide_clf.osim"
    osim_path.write_text(
        _build_osim_v4_xml(
            bodies_xml=bodies_xml, joints_xml=joints_xml, forces_xml=forces_xml
        ),
        encoding="utf-8",
    )

    out_xml = convert(str(osim_path), str(tmp_path / "out"))
    tree = etree.parse(out_xml)
    slide_jnt = tree.find(".//joint[@name='rail_tx']")
    assert slide_jnt is not None
    assert slide_jnt.attrib["type"] == "slide"
    assert slide_jnt.attrib["limited"] == "true"

    rng = [float(v) for v in slide_jnt.attrib["range"].split()]
    np.testing.assert_allclose(rng, [-0.05, 0.12], atol=1e-6)
    np.testing.assert_allclose(
        [float(v) for v in slide_jnt.attrib["solreflimit"].split()],
        [-400.0, -15.0],
        atol=1e-6,
    )
    np.testing.assert_allclose(
        [float(v) for v in slide_jnt.attrib["solimplimit"].split()],
        [0.0001, 0.99, 0.008, 0.5, 1.0],
        atol=1e-6,
    )

    model = mujoco.MjModel.from_xml_path(out_xml)
    data = mujoco.MjData(model)
    data.qpos[0] = 0.16
    mujoco.mj_forward(model, data)
    assert data.qfrc_constraint[0] < -1.0

    data.qpos[0] = -0.09
    mujoco.mj_forward(model, data)
    assert data.qfrc_constraint[0] > 1.0
