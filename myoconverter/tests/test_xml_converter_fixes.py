"""Regression tests for XML converter bug fixes (#36, #38, #43, and parser edge cases)."""

import numpy as np
import pytest
from lxml import etree
from scipy.spatial.transform import Rotation

from myoconverter.xml.converter import convert
from myoconverter.xml.utils import calculate_mujoco_position, calculate_mujoco_transformation, str2vec
from myoconverter.xml.wrap_objects.WrapSphere import WrapSphere


def _minimal_osim_xml(
    *,
    bodies_xml: str,
    joints_xml: str,
    forces_xml: str = "",
    constraints_xml: str = "",
    credits: str = "Unit Test Model",
) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
  <Model name="test_model">
    <credits>{credits}</credits>
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
    <ConstraintSet name="constraintset">
      <objects>
        {constraints_xml}
      </objects>
      <groups />
    </ConstraintSet>
    <ForceSet name="forceset">
      <objects>
        {forces_xml}
      </objects>
      <groups />
    </ForceSet>
    <MarkerSet name="markerset">
      <objects />
      <groups />
    </MarkerSet>
  </Model>
</OpenSimDocument>
"""


def _body_xml(name: str, wrap_xml: str = "") -> str:
    return f"""
    <Body name="{name}">
      <mass>1.0</mass>
      <mass_center>0 0 0</mass_center>
      <inertia>0.01 0.01 0.01 0 0 0</inertia>
      <WrapObjectSet name="wrapobjectset">
        <objects>{wrap_xml}</objects>
        <groups />
      </WrapObjectSet>
    </Body>
    """


def _pin_joint_xml(
    name: str,
    parent_body: str,
    child_body: str,
    coord_name: str,
    p_trans: str = "0 0 0",
    p_orient: str = "0 0 0",
    c_trans: str = "0 0 0",
    c_orient: str = "0 0 0",
) -> str:
    parent_path = "/ground" if parent_body == "ground" else f"/bodyset/{parent_body}"
    return f"""
    <PinJoint name="{name}">
      <socket_parent_frame>{parent_body}_offset</socket_parent_frame>
      <socket_child_frame>{child_body}_offset</socket_child_frame>
      <coordinates>
        <Coordinate name="{coord_name}">
          <default_value>0</default_value>
          <range>-1.57 1.57</range>
          <clamped>true</clamped>
          <locked>false</locked>
        </Coordinate>
      </coordinates>
      <frames>
        <PhysicalOffsetFrame name="{parent_body}_offset">
          <socket_parent>{parent_path}</socket_parent>
          <translation>{p_trans}</translation>
          <orientation>{p_orient}</orientation>
        </PhysicalOffsetFrame>
        <PhysicalOffsetFrame name="{child_body}_offset">
          <socket_parent>/bodyset/{child_body}</socket_parent>
          <translation>{c_trans}</translation>
          <orientation>{c_orient}</orientation>
        </PhysicalOffsetFrame>
      </frames>
    </PinJoint>
    """


def test_converter_no_infinite_recursion_when_outgoing_joint_precedes_incoming(tmp_path):
    """Regression test for Issue #38: parent_name mutation in _add_bodies_and_joints loop."""
    bodies = _body_xml("pelvis") + _body_xml("femur_r") + _body_xml("femur_l")
    # Outgoing joints of pelvis (hip_r, hip_l) appear BEFORE ground_pelvis, and hip_r precedes ground_pelvis
    joints = (
        _pin_joint_xml("hip_r", "pelvis", "femur_r", "hip_flex_r")
        + _pin_joint_xml("ground_pelvis", "ground", "pelvis", "pelvis_tilt")
        + _pin_joint_xml("hip_l", "pelvis", "femur_l", "hip_flex_l")
    )
    osim_file = tmp_path / "branch_order.osim"
    osim_file.write_text(_minimal_osim_xml(bodies_xml=bodies, joints_xml=joints), encoding="utf-8")

    out_xml = convert(str(osim_file), str(tmp_path / "out"))
    tree = etree.parse(out_xml)
    pelvis = tree.find(".//worldbody/body[@name='pelvis']")
    assert pelvis is not None
    assert pelvis.find("body[@name='femur_r']") is not None
    assert pelvis.find("body[@name='femur_l']") is not None
    assert len(tree.findall(".//body[@name='pelvis']")) == 1


def test_child_frame_se3_transform_and_joint_pivot_offset(tmp_path):
    """Regression test for Issue #43: T_{Bp,Bc} = T_{Bp,Fp} * inv(T_{Bc,Fc}) and joint pos = p_C."""
    p_t = np.array([0.10, 0.20, -0.30])
    p_o = np.array([0.25, -0.40, 0.60])
    c_t = np.array([0.04, -0.05, 0.06])
    c_o = np.array([-0.35, 0.30, 0.15])

    bodies = _body_xml("parent_b") + _body_xml("child_b")
    joints = _pin_joint_xml("root_j", "ground", "parent_b", "q0") + _pin_joint_xml(
        "child_j",
        "parent_b",
        "child_b",
        "q1",
        p_trans=" ".join(map(str, p_t)),
        p_orient=" ".join(map(str, p_o)),
        c_trans=" ".join(map(str, c_t)),
        c_orient=" ".join(map(str, c_o)),
    )
    osim_file = tmp_path / "se3_model.osim"
    osim_file.write_text(
        _minimal_osim_xml(bodies_xml=bodies, joints_xml=joints, credits="Credits -- with double hyphens -"),
        encoding="utf-8",
    )

    out_xml = convert(str(osim_file), str(tmp_path / "out"))
    tree = etree.parse(out_xml)
    child_body = tree.find(".//body[@name='child_b']")
    child_joint = child_body.find("joint[@name='q1']")

    # Expected SE(3) relative body pose: R = R_P * R_C^{-1}, p = p_P - R * p_C
    r_p = Rotation.from_euler("XYZ", p_o)
    r_c = Rotation.from_euler("XYZ", c_o)
    r_rel = r_p * r_c.inv()
    expected_pos = p_t - r_rel.apply(c_t)
    expected_euler = r_rel.as_euler("XYZ")

    actual_pos = str2vec(child_body.attrib["pos"])
    actual_euler = str2vec(child_body.attrib["euler"])
    actual_joint_pos = str2vec(child_joint.attrib["pos"])
    actual_joint_axis = str2vec(child_joint.attrib["axis"])

    np.testing.assert_allclose(actual_pos, expected_pos, atol=1e-3)
    np.testing.assert_allclose(
        Rotation.from_euler("XYZ", actual_euler).as_matrix(),
        r_rel.as_matrix(),
        atol=2e-3,
    )
    np.testing.assert_allclose(actual_joint_pos, c_t, atol=1e-4)

    # Verify full forward kinematics at non-zero joint angle theta matches OpenSim SE(3) chain
    theta = 0.45
    t_p = np.eye(4)
    t_p[:3, :3] = r_p.as_matrix()
    t_p[:3, 3] = p_t
    t_c = np.eye(4)
    t_c[:3, :3] = r_c.as_matrix()
    t_c[:3, 3] = c_t
    t_j = np.eye(4)
    t_j[:3, :3] = Rotation.from_rotvec(np.array([0.0, 0.0, 1.0]) * theta).as_matrix()
    t_osim = t_p @ t_j @ np.linalg.inv(t_c)

    t_body_mjc = np.eye(4)
    t_body_mjc[:3, :3] = Rotation.from_euler("XYZ", actual_euler).as_matrix()
    t_body_mjc[:3, 3] = actual_pos
    t_hinge_mjc = np.eye(4)
    r_hinge = Rotation.from_rotvec(actual_joint_axis * theta).as_matrix()
    t_hinge_mjc[:3, :3] = r_hinge
    t_hinge_mjc[:3, 3] = actual_joint_pos - r_hinge @ actual_joint_pos
    t_mjc = t_body_mjc @ t_hinge_mjc

    np.testing.assert_allclose(t_mjc, t_osim, atol=2e-3)


def test_wrap_sidesite_naming_and_sphere_negative_quadrant(tmp_path):
    """Verify WrapSphere negative quadrant signs and default sidesite lookup in PathWrapSet."""
    wrap_xml = """
    <WrapSphere name="sph_wrap">
      <active>true</active>
      <xyz_body_rotation>0 0 0</xyz_body_rotation>
      <translation>0 0 0</translation>
      <quadrant>-y</quadrant>
      <radius>0.05</radius>
    </WrapSphere>
    """
    forces_xml = """
    <Thelen2003Muscle name="m1">
      <appliesForce>true</appliesForce>
      <max_isometric_force>100</max_isometric_force>
      <optimal_fiber_length>0.1</optimal_fiber_length>
      <tendon_slack_length>0.1</tendon_slack_length>
      <pennation_angle_at_optimal>0</pennation_angle_at_optimal>
      <min_control>0.1</min_control>
      <max_control>0.8</max_control>
      <GeometryPath name="gp">
        <PathPointSet>
          <objects>
            <PathPoint name="p1">
              <socket_parent_frame>/bodyset/b1</socket_parent_frame>
              <location>-0.1 0.02 0</location>
            </PathPoint>
            <PathPoint name="p2">
              <socket_parent_frame>/bodyset/b1</socket_parent_frame>
              <location>0.1 0.02 0</location>
            </PathPoint>
          </objects>
        </PathPointSet>
        <PathWrapSet>
          <objects>
            <PathWrap name="pw">
              <wrap_object>sph_wrap</wrap_object>
              <method>hybrid</method>
              <range>-1 -1</range>
            </PathWrap>
          </objects>
        </PathWrapSet>
      </GeometryPath>
    </Thelen2003Muscle>
    <Thelen2003Muscle name="m2">
      <appliesForce>true</appliesForce>
      <max_isometric_force>120</max_isometric_force>
      <optimal_fiber_length>0.1</optimal_fiber_length>
      <tendon_slack_length>0.1</tendon_slack_length>
      <pennation_angle_at_optimal>0</pennation_angle_at_optimal>
      <GeometryPath name="gp2">
        <PathPointSet>
          <objects>
            <PathPoint name="p1">
              <socket_parent_frame>/bodyset/b1</socket_parent_frame>
              <location>0 0 0</location>
            </PathPoint>
            <PathPoint name="p2">
              <socket_parent_frame>/bodyset/b1</socket_parent_frame>
              <location>0 0.1 0</location>
            </PathPoint>
          </objects>
        </PathPointSet>
        <PathWrapSet><objects /></PathWrapSet>
      </GeometryPath>
    </Thelen2003Muscle>
    <CoordinateActuator name="disabled_act">
      <appliesForce>false</appliesForce>
      <coordinate>q0</coordinate>
      <optimal_force>10</optimal_force>
    </CoordinateActuator>
    <CoordinateActuator name="missing_joint_act">
      <appliesForce>true</appliesForce>
      <coordinate>nonexistent_coord</coordinate>
      <optimal_force>10</optimal_force>
    </CoordinateActuator>
    <CoordinateActuator name="valid_act">
      <appliesForce>true</appliesForce>
      <coordinate>q0</coordinate>
      <optimal_force>10</optimal_force>
    </CoordinateActuator>
    """
    osim_file = tmp_path / "wrap_muscle.osim"
    osim_file.write_text(
        _minimal_osim_xml(
            bodies_xml=_body_xml("b1", wrap_xml=wrap_xml),
            joints_xml=_pin_joint_xml("j0", "ground", "b1", "q0"),
            forces_xml=forces_xml,
        ),
        encoding="utf-8",
    )

    out_xml = convert(str(osim_file), str(tmp_path / "out"))
    tree = etree.parse(out_xml)

    # Default wrap sidesite must include _{osim_type} and be placed at -(0.05 + 0.001) = -0.051 along Y
    default_site = tree.find(".//body[@name='b1']/site[@name='sph_wrap_sphere_sidesite']")
    assert default_site is not None
    expected_y = -(0.05 + WrapSphere.sidesite_dist())
    np.testing.assert_allclose(str2vec(default_site.attrib["pos"]), [0.0, expected_y, 0.0], atol=1e-4)

    # Tendon wrap sidesite must pick up the default quadrant sidesite (-0.051 Y), not +Y segment midpoint
    tendon_site = tree.find(".//body[@name='b1']/site[@name='sph_wrap_sphere_m1_1_sidesite']")
    assert tendon_site is not None
    np.testing.assert_allclose(str2vec(tendon_site.attrib["pos"]), [0.0, expected_y, 0.0], atol=1e-4)

    # Muscle ctrlrange must NOT bleed from m1 onto m2
    m1_el = tree.find(".//actuator/muscle[@name='m1']")
    m2_el = tree.find(".//actuator/muscle[@name='m2']")
    assert m1_el.attrib.get("ctrlrange") == "0.1 0.8"
    assert "ctrlrange" not in m2_el.attrib

    # CoordinateActuator: disabled & missing-joint skipped; valid motor placed before muscles
    motors = tree.findall(".//actuator/motor")
    assert [m.attrib["name"] for m in motors] == ["valid_act"]
    actuator_elems = [el for el in tree.find(".//actuator") if isinstance(el.tag, str)]
    assert actuator_elems[0].attrib["name"] == "valid_act"


def test_coupler_constraint_validation_and_moving_path_point_constant_axis(tmp_path):
    """Test CoordinateCouplerConstraint 2-coord rejection, optional isEnforced, and Constant MovingPathPoint axis."""
    constraints_ok = """
    <CoordinateCouplerConstraint name="coupler_ok">
      <dependent_coordinate_name>q1</dependent_coordinate_name>
      <independent_coordinate_names>q0</independent_coordinate_names>
      <coupled_coordinates_function>
        <LinearFunction><coefficients>0.5 0</coefficients></LinearFunction>
      </coupled_coordinates_function>
    </CoordinateCouplerConstraint>
    """
    forces_mpp = """
    <Thelen2003Muscle name="m_mpp">
      <appliesForce>true</appliesForce>
      <max_isometric_force>100</max_isometric_force>
      <optimal_fiber_length>0.05</optimal_fiber_length>
      <tendon_slack_length>0.05</tendon_slack_length>
      <pennation_angle_at_optimal>0</pennation_angle_at_optimal>
      <GeometryPath name="gp">
        <PathPointSet>
          <objects>
            <MovingPathPoint name="mpp1">
              <socket_parent_frame>/bodyset/b1</socket_parent_frame>
              <socket_x_coordinate>/jointset/j0/q0</socket_x_coordinate>
              <socket_y_coordinate>/jointset/j0/q0</socket_y_coordinate>
              <socket_z_coordinate>/jointset/j0/q0</socket_z_coordinate>
              <x_location>
                <SimmSpline>
                  <x>-1.0 0.0 1.0</x>
                  <y>0.01 0.02 0.03</y>
                </SimmSpline>
              </x_location>
              <y_location>
                <Constant><value>0.04</value></Constant>
              </y_location>
              <z_location>
                <Constant><value>-0.02</value></Constant>
              </z_location>
            </MovingPathPoint>
            <PathPoint name="p2">
              <socket_parent_frame>/bodyset/b2</socket_parent_frame>
              <location>0 0 0</location>
            </PathPoint>
          </objects>
        </PathPointSet>
        <PathWrapSet><objects /></PathWrapSet>
      </GeometryPath>
    </Thelen2003Muscle>
    """
    bodies = _body_xml("b1") + _body_xml("b2")
    joints = _pin_joint_xml("j0", "ground", "b1", "q0") + _pin_joint_xml("j1", "b1", "b2", "q1")
    osim_file = tmp_path / "coupler_mpp.osim"
    osim_file.write_text(
        _minimal_osim_xml(bodies_xml=bodies, joints_xml=joints, constraints_xml=constraints_ok, forces_xml=forces_mpp),
        encoding="utf-8",
    )

    out_xml = convert(str(osim_file), str(tmp_path / "out"))
    tree = etree.parse(out_xml)
    eq = tree.find(".//equality/joint[@name='coupler_ok']")
    assert eq is not None
    assert eq.attrib["active"] == "true"

    mpp_body = tree.find(".//body[@name='m_mpp_mpp1']")
    assert mpp_body is not None
    np.testing.assert_allclose(str2vec(mpp_body.attrib["pos"]), [0.0, 0.04, -0.02], atol=1e-4)

    # Two independent coordinates must raise RuntimeError
    constraints_bad = constraints_ok.replace(
        "<independent_coordinate_names>q0</independent_coordinate_names>",
        "<independent_coordinate_names>q0 q1</independent_coordinate_names>",
    )
    bad_file = tmp_path / "bad_coupler.osim"
    bad_file.write_text(
        _minimal_osim_xml(bodies_xml=bodies, joints_xml=joints, constraints_xml=constraints_bad),
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError):
        convert(str(bad_file), str(tmp_path / "out_bad"))


def test_calculate_mujoco_position_includes_body_rotations():
    """Verify calculate_mujoco_position accounts for euler/quat on ancestor bodies."""
    worldbody = etree.fromstring(
        """
        <worldbody>
          <body name="root" pos="1 0 0" euler="1.5707963267948966 0 0">
            <body name="child" pos="0 1 0">
              <site name="tip" pos="0 0 2"/>
            </body>
          </body>
        </worldbody>
        """
    )
    # R_x(90 deg) maps [0, 1, 2] -> [0, -2, 1]; plus root pos [1, 0, 0] -> [1, -2, 1]
    pos = calculate_mujoco_position("site", "tip", worldbody)
    np.testing.assert_allclose(pos, [1.0, -2.0, 1.0], atol=1e-6)
    T = calculate_mujoco_transformation("body", "child", worldbody)
    np.testing.assert_allclose(T[:3, 3], [1.0, 0.0, 1.0], atol=1e-6)


def test_custom_joint_unused_coordinate_and_case_insensitive_stl(tmp_path):
    """Verify CustomJoint with an unused Coordinate (no equality constraint) and uppercase .STL mesh."""
    geom_dir = tmp_path / "Geometry"
    geom_dir.mkdir()
    stl_content = "solid cube\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid cube\n"
    (geom_dir / "part.STL").write_text(stl_content, encoding="utf-8")

    body_xml = """
    <Body name="b1">
      <mass>1.0</mass>
      <mass_center>0 0 0</mass_center>
      <inertia>0.01 0.01 0.01 0 0 0</inertia>
      <attached_geometry>
        <Mesh name="b1_mesh">
          <scale_factors>1 1 1</scale_factors>
          <mesh_file>part.STL</mesh_file>
        </Mesh>
      </attached_geometry>
      <WrapObjectSet name="wrapobjectset"><objects /><groups /></WrapObjectSet>
    </Body>
    """
    custom_joint_xml = """
    <CustomJoint name="cj">
      <socket_parent_frame>ground_offset</socket_parent_frame>
      <socket_child_frame>b1_offset</socket_child_frame>
      <coordinates>
        <Coordinate name="used_q">
          <default_value>0</default_value>
          <range>-1 1</range>
          <clamped>true</clamped>
          <locked>false</locked>
        </Coordinate>
        <Coordinate name="unused_q">
          <default_value>0</default_value>
          <range>-1 1</range>
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
        <PhysicalOffsetFrame name="b1_offset">
          <socket_parent>/bodyset/b1</socket_parent>
          <translation>0 0 0</translation>
          <orientation>0 0 0</orientation>
        </PhysicalOffsetFrame>
      </frames>
      <SpatialTransform>
        <TransformAxis name="rotation1">
          <coordinates>used_q</coordinates>
          <axis>0 0 1</axis>
          <LinearFunction><coefficients>1 0</coefficients></LinearFunction>
        </TransformAxis>
        <TransformAxis name="rotation2"><axis>1 0 0</axis><Constant><value>0</value></Constant></TransformAxis>
        <TransformAxis name="rotation3"><axis>0 1 0</axis><Constant><value>0</value></Constant></TransformAxis>
        <TransformAxis name="translation1"><axis>1 0 0</axis><Constant><value>0</value></Constant></TransformAxis>
        <TransformAxis name="translation2"><axis>0 1 0</axis><Constant><value>0</value></Constant></TransformAxis>
        <TransformAxis name="translation3"><axis>0 0 1</axis><Constant><value>0</value></Constant></TransformAxis>
      </SpatialTransform>
    </CustomJoint>
    """
    osim_file = tmp_path / "custom.osim"
    osim_file.write_text(_minimal_osim_xml(bodies_xml=body_xml, joints_xml=custom_joint_xml), encoding="utf-8")

    out_xml = convert(str(osim_file), str(tmp_path / "out"), geometry_folder=str(geom_dir))
    tree = etree.parse(out_xml)
    mesh_el = tree.find(".//asset/mesh[@name='b1_mesh_part']")
    assert mesh_el is not None
    assert mesh_el.attrib["file"] == "Geometry/part.stl"
