"""Regression tests for optimization utilities and conversion steps (Steps 1-3)."""

import os
import pickle
import sys
import types
from unittest.mock import MagicMock, patch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pytest

# Allow running tests in environments without OpenSim installed (while using real OpenSim if present).
try:
    import opensim  # noqa: F401
except ModuleNotFoundError:
    _osim_stub = types.ModuleType("opensim")
    _osim_stub.Model = MagicMock
    _osim_stub.CoordinateCouplerConstraint = MagicMock()
    sys.modules["opensim"] = _osim_stub

from myoconverter.conversion_steps.O2MStep1 import BasicModelConvert
from myoconverter.conversion_steps.O2MStep2 import MomentArmOpt
from myoconverter.conversion_steps.O2MStep3 import MuscleForceOpt
from myoconverter.conversion_steps.O2MSteps import O2MSteps
from myoconverter.optimization.model_states.MjcMuscleStates import MjcMuscleStates
from myoconverter.optimization.model_states.OsimMuscleStates import OsimMuscleStates
from myoconverter.optimization.utils.UtilsForceOpt import (
    fmOptPSO_cust,
    getMuscleForceDiff,
    initializeWorker,
    objFMMuscle,
)
from myoconverter.optimization.utils.UtilsLengthOpt import getMomentArmDiff
from myoconverter.optimization.utils.UtilsMujoco import (
    _get_actuator_moment,
    computeMomentArmMuscleJoints,
    dependencyJointAng,
    getCoordinateRange_mjc,
    getMuscleForceLengthCurvesSim,
    sortMuscleWrapSiteJoint,
    updateMuscleForceProperties,
)
from myoconverter.optimization.utils.UtilsOpensim import getIndipCoordAndJoint
from myoconverter.optimization.utils.UtilsRotation import (
    TaitBryanRotationMatrix,
    cartesian2spherical,
    euler_change_sequence,
    euler_change_sequence_bodyRotationFirst,
    spherical2cartesian,
)


SYNTHETIC_MJC_XML = """\
<mujoco model="test_arm">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="upper_arm" pos="0 0 0">
      <inertial pos="0 0 -0.15" mass="2.0" diaginertia="0.01 0.01 0.002"/>
      <joint name="shoulder_elv" type="hinge" axis="0 0 1" limited="true" range="-0.5 1.0"/>
      <site name="bic_origin" pos="0 0.03 0"/>
      <site name="wrap_cyl_sidesite" pos="0 0.05 -0.3"/>
      <geom name="wrap_cylinder" type="cylinder" pos="0 0 -0.3" size="0.03 0.05" contype="0" conaffinity="0"/>
      <body name="forearm" pos="0 0 -0.3">
        <inertial pos="0 0 -0.15" mass="1.5" diaginertia="0.008 0.008 0.001"/>
        <joint name="elbow_flex" type="hinge" axis="0 0 1" limited="true" range="0.0 2.0"/>
        <joint name="elbow_coupled" type="hinge" axis="1 0 0" limited="true" range="-1.0 1.0"/>
        <site name="bic_insert" pos="0.04 0.02 -0.1"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <joint name="elbow_coupler" joint1="elbow_coupled" joint2="elbow_flex" polycoef="0.1 0.5 0 0 0" active="true"/>
  </equality>
  <tendon>
    <spatial name="bic_tendon">
      <site site="bic_origin"/>
      <geom geom="wrap_cylinder" sidesite="wrap_cyl_sidesite"/>
      <site site="bic_insert"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="bic_long" tendon="bic_tendon" lengthrange="0.28 0.36"
            range="0.75 1.05" force="200" scale="200" lmin="0.5" lmax="1.6" vmax="1.5" fpmax="1.3" fvmax="1.2"/>
  </actuator>
</mujoco>
"""


def _make_mock_osim_model(muscle_names=("bic_long",)):
    model = MagicMock()
    muscles = MagicMock()
    muscles.getSize.return_value = len(muscle_names)

    def _get_muscle(key):
        m = MagicMock()
        if isinstance(key, int):
            m.getName.return_value = muscle_names[key]
        else:
            m.getName.return_value = key
        return m

    muscles.get.side_effect = _get_muscle
    model.getMuscles.return_value = muscles
    return model


def test_utils_mujoco_no_timestep_mutation_and_steady_activation():
    """Step 2/3 evaluations must not mutate model.opt.timestep or suffer integration drift (#41)."""
    model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)
    assert model.opt.timestep == pytest.approx(0.002)

    ma = computeMomentArmMuscleJoints(model, "bic_long", ["elbow_flex"], [[0.0, 1.5]], 4)
    assert ma.shape == (4, 1)
    assert model.opt.timestep == pytest.approx(0.002)

    jnt_sets = [(0.2,), (0.8,), (1.4,)]
    f_passive, l_passive = getMuscleForceLengthCurvesSim(model, "bic_long", ["elbow_flex"], jnt_sets, [0.0])
    f_active, l_active = getMuscleForceLengthCurvesSim(model, "bic_long", ["elbow_flex"], jnt_sets, [1.0])

    assert model.opt.timestep == pytest.approx(0.002)
    assert np.allclose(l_passive, l_active)
    # MuJoCo muscle actuator forces are negative (pulling); active magnitude must exceed passive magnitude.
    assert np.all(np.abs(f_active) > np.abs(f_passive))


def test_utils_mujoco_sidesite_zero_and_wrap_info_isolation():
    """sidesite ID 0 must be recognized (wrap_prm >= 0), and wrapInfo must not alias across joint groups."""
    xml_site0 = """\
    <mujoco model="site0_test">
      <worldbody>
        <body name="b1" pos="0 0 0">
          <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
          <joint name="j1" type="hinge" axis="1 0 0" limited="true" range="-1 1"/>
          <site name="side0" pos="0 0.06 -0.1"/>
          <site name="orig" pos="0 0.01 0"/>
          <geom name="cyl_wrap" type="cylinder" euler="0 1.5708 0" pos="0 0 -0.1" size="0.04 0.05" contype="0" conaffinity="0"/>
          <body name="b2" pos="0 0 -0.1">
            <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
            <joint name="j2" type="hinge" axis="1 0 0" limited="true" range="-1 1"/>
            <site name="ins" pos="0 0.01 -0.1"/>
          </body>
        </body>
      </worldbody>
      <tendon>
        <spatial name="m1_tendon">
          <site site="orig"/>
          <geom geom="cyl_wrap" sidesite="side0"/>
          <site site="ins"/>
        </spatial>
      </tendon>
      <actuator>
        <muscle name="m1" tendon="m1_tendon" lengthrange="0.15 0.35"/>
      </actuator>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml_site0)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "side0") == 0

    muscle_para = {
        "muscle_name": "m1",
        "wrapping_coordinates": [["j2"], ["j1"]],
        "mjc_coordinate_ranges": [[[-0.5, 0.5]], [[-0.5, 0.5]]],
    }
    res = sortMuscleWrapSiteJoint(model, muscle_para)
    wrap_j2 = res["wrapping_info"][0]
    assert wrap_j2 is not None and len(wrap_j2) == 7
    assert wrap_j2[6][0] == 0  # site ID 0 preserved!


def test_dependency_joint_ang_with_list_free_jnt_ids():
    """dependencyJointAng must work when free_jnt_id_array is a Python list (NumPy 2 compatibility)."""
    model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)
    _, free_jnt_ids = getCoordinateRange_mjc(model)
    assert isinstance(free_jnt_ids, list)

    elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow_flex")
    coupled_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow_coupled")
    free_angles = [1.0 if jid == elbow_id else 0.0 for jid in free_jnt_ids]
    dep_jnts, dep_angles, _ = dependencyJointAng(model, free_jnt_ids, free_angles)

    assert coupled_id in dep_jnts
    coupled_val = dep_angles[dep_jnts.index(coupled_id)]
    assert coupled_val == pytest.approx(0.1 + 0.5 * 1.0)


def test_mjc_muscle_states_uses_modern_mujoco_api():
    """MjcMuscleStates must use mujoco.mj_id2name (not mujoco_py attributes) and valid imports."""
    model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)
    states = MjcMuscleStates(model, {"bic_long": [["elbow_flex"]]}, muscle_list=None)
    wrap_coords = states.getWrapingCoords()
    assert "bic_long" in wrap_coords
    ma = states.getMomentArms(evalN=4)
    assert "bic_long" in ma
    assert ma["bic_long"][0].shape == (25, 1)


def test_utils_rotation_spherical_and_euler_change_sequence():
    """cartesian2spherical must take sqrt(r^2), and euler_change_sequence must not index rot[3][1]."""
    centre = np.array([1.0, -2.0, 0.5])
    pose = centre + np.array([3.0, 4.0, 12.0])
    sph = cartesian2spherical(centre, pose)
    assert sph[0] == pytest.approx(13.0)
    cart = spherical2cartesian(centre, sph)
    assert np.allclose(cart, pose)

    angles_xzy = euler_change_sequence("xyz", [0.1, 0.2, 0.3], "xzy")
    assert len(angles_xzy) == 3
    assert np.allclose(
        TaitBryanRotationMatrix("xzy", angles_xzy),
        TaitBryanRotationMatrix("xyz", [0.1, 0.2, 0.3]),
        atol=1e-6,
    )

    angles_body_xzy = euler_change_sequence_bodyRotationFirst("xyz", [0.05, 0.1, -0.05], "xyz", [0.1, 0.2, 0.3], "xzy")
    assert len(angles_body_xzy) == 3


def test_force_and_length_diff_exceptions_and_empty_error_indices():
    """getMuscleForceDiff must return falsy [] when within threshold, and raise ValueError on bad inputs."""
    model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)
    jnt_sets = [(0.2,), (0.8,)]
    f_mjc, _ = getMuscleForceLengthCurvesSim(model, "bic_long", ["elbow_flex"], jnt_sets, [1.0])
    # Matching normalized force (note opposite sign convention between OpenSim and MuJoCo)
    fmax = model.actuator("bic_long").gainprm[2]
    # In getMuscleForceDiff, mjc forces are raw actuator_force when gainprm[2]==1 or scaled; pass matching array:
    err_ind, _, cost = getMuscleForceDiff(model, "bic_long", ["elbow_flex"], jnt_sets, [1.0], -f_mjc)
    assert err_ind == []
    assert not bool(err_ind)
    assert cost == pytest.approx(0.0, abs=1e-12)

    with pytest.raises(ValueError):
        getMuscleForceDiff(model, "bic_long", ["elbow_flex"], jnt_sets, [1.0], np.zeros((1, 5)))

    with pytest.raises(ValueError):
        getMomentArmDiff(model, "bic_long", ["elbow_flex"], [[0.0, 1.0]], np.zeros((10, 1)), 4)


def test_utils_opensim_indip_coord_and_osim_muscle_states_none():
    """getIndipCoordAndJoint must guard safeDownCast None, multi-coord couplers, and unrelated couplers."""
    other_constraint = MagicMock()
    coupler_unrelated = MagicMock()
    coupler_unrelated.getDependentCoordinateName.return_value = "other_coord"

    coupler_single = MagicMock()
    coupler_single.getDependentCoordinateName.return_value = "elbow_coupled"
    ind_single = MagicMock()
    ind_single.getSize.return_value = 1
    ind_single.get.return_value = "elbow_flex"
    coupler_single.getIndependentCoordinateNames.return_value = ind_single

    coupler_multi = MagicMock()
    coupler_multi.getDependentCoordinateName.return_value = "multi_coupled"
    ind_multi = MagicMock()
    ind_multi.getSize.return_value = 2
    coupler_multi.getIndependentCoordinateNames.return_value = ind_multi

    constraints = [other_constraint, coupler_unrelated, coupler_single, coupler_multi]
    cset = MagicMock()
    cset.getSize.return_value = len(constraints)
    cset.get.side_effect = lambda idx: constraints[idx]

    coord_map = {}
    for cname, jname in [
        ("elbow_coupled", "j_coupled"),
        ("elbow_flex", "j_flex"),
        ("multi_coupled", "j_multi"),
        ("unmatched_coupled", "j_unmatched"),
    ]:
        c = MagicMock()
        c.isConstrained.return_value = True
        j = MagicMock()
        j.getName.return_value = jname
        c.getJoint.return_value = j
        coord_map[cname] = c

    cs = MagicMock()
    cs.get.side_effect = lambda name: coord_map[name]

    model = MagicMock()
    model.getConstraintSet.return_value = cset
    model.getCoordinateSet.return_value = cs
    model.initSystem.return_value = MagicMock()

    def _safe_down_cast(obj):
        if obj is other_constraint:
            return None
        return obj

    with patch(
        "myoconverter.optimization.utils.UtilsOpensim.opensim.CoordinateCouplerConstraint.safeDownCast",
        side_effect=_safe_down_cast,
    ):
        assert getIndipCoordAndJoint(model, "elbow_coupled") == ["elbow_flex", "j_flex"]
        assert getIndipCoordAndJoint(model, "multi_coupled") == [[], []]
        assert getIndipCoordAndJoint(model, "unmatched_coupled") == [[], []]

    mock_osim = _make_mock_osim_model(("m_a", "m_b"))
    states = OsimMuscleStates(mock_osim, muscle_list=None)
    assert states.muscle_list == ["m_a", "m_b"]


def test_o2msteps_and_step1_validation_and_endpoint_plots(tmp_path):
    """O2MSteps and BasicModelConvert must raise standard exceptions and compute L2 marker error & boxplots."""
    steps = O2MSteps(
        osim_model_file=str(tmp_path / "missing.osim"),
        geom_folder=str(tmp_path),
        mjc_target_folder=str(tmp_path / "out"),
        convert_steps=[2],
        convert=True,
    )
    with pytest.raises(FileNotFoundError):
        steps.PipelineExecution()

    with pytest.raises(FileNotFoundError):
        BasicModelConvert(str(tmp_path / "nonexistent.osim"), str(tmp_path), str(tmp_path / "step1"))

    osim_dummy = tmp_path / "dummy.osim"
    osim_dummy.write_text("<OpenSimDocument/>", encoding="utf-8")
    with pytest.raises(FileNotFoundError):
        BasicModelConvert(str(osim_dummy), str(tmp_path / "missing_geom"), str(tmp_path / "step1"))

    step1_dir = tmp_path / "step1"
    conv = BasicModelConvert(str(osim_dummy), str(tmp_path), str(step1_dir))
    with pytest.raises(ValueError):
        conv.vlt1_forwardKinematicsValidation(end_points_osim=["mk_0"])

    # Zero-marker case: must not emit NaN mean/std
    os.makedirs(step1_dir / "end_points", exist_ok=True)
    conv.endPointsPlot([], [], [])
    with open(step1_dir / "end_points" / "end_point_error.pkl", "rb") as f:
        err0 = pickle.load(f)
    assert err0["mean"] == 0.0
    assert err0["std"] == 0.0

    # 10-marker case (marNum % 10 == 0) + Euclidean L2 check:
    # OpenSim (x, y, z) vs MuJoCo (x, -z, y): delta = (0.03, 0.04, 0.0) -> L2 = 0.05 (L1 was 0.07)
    names_10 = [f"mk_{i}" for i in range(10)]
    end_pos_osim = [np.tile([0.03, 0.04, 0.0], (10, 1)) for _ in range(3)]
    end_pos_mjc = [np.zeros((10, 3)) for _ in range(3)]

    with patch.object(plt.Figure, "savefig"):
        conv.endPointsPlot(end_pos_osim, end_pos_mjc, names_10)

    with open(step1_dir / "end_points" / "end_point_error.pkl", "rb") as f:
        err10 = pickle.load(f)
    assert err10["mean"] == pytest.approx(0.05)


def test_step2_torus_dict_and_pickle_roundtrip(tmp_path):
    """MomentArmOpt must store dict for torus wraps, skip non-muscle pkls, and overwrite muscle pkls cleanly."""
    cvt1_path = tmp_path / "test_cvt1.xml"
    cvt2_path = tmp_path / "test_cvt2.xml"
    cvt1_path.write_text(SYNTHETIC_MJC_XML, encoding="utf-8")
    cvt2_path.write_text(SYNTHETIC_MJC_XML, encoding="utf-8")

    save_dir = tmp_path / "Step2_muscleKinematics"
    save_dir.mkdir()

    # Pre-populate muscle pickle + non-muscle config.pkl
    with open(save_dir / "config.pkl", "wb") as f:
        pickle.dump({"not_a_muscle": True}, f)

    mus_pkl = {
        "muscle_name": "bic_long",
        "wrapping_coordinates": [["elbow_flex", "virtual_extra_jnt"]],
        "osim_coordinate_ranges": [[[0.0, 1.5], [-0.5, 0.5]]],
        "mjc_coordinate_ranges": [[[0.0, 1.5], [-0.5, 0.5]]],
        "osim_ma_data": [np.zeros((9, 2))],
        "evalN": [3],
        "opt_results": [{"cost_org": 0.0, "cost_opt": 0.0, "par_opt": []}],
    }
    # Use a 1-joint setup for real MuJoCo evaluation + an extra joint test
    mus_pkl_1j = {
        "muscle_name": "bic_long",
        "wrapping_coordinates": [["elbow_flex"]],
        "osim_coordinate_ranges": [[[0.0, 1.5]]],
        "mjc_coordinate_ranges": [[[0.0, 1.5]]],
        "osim_ma_data": [np.ones((3, 1)) * 0.5],  # large diff so err_ind is non-empty
        "evalN": [3],
    }
    with open(save_dir / "bic_long.pkl", "wb") as f:
        pickle.dump(mus_pkl_1j, f)

    mock_osim = _make_mock_osim_model(("bic_long",))
    with patch("myoconverter.conversion_steps.O2MStep2.opensim.Model", return_value=mock_osim):
        ma_opt = MomentArmOpt(str(cvt1_path), "dummy.osim", str(save_dir), muscle_list=None, osim_data_overwrite=False)

    # Simulate torus wrap object in sortMuscleWrapSiteJoint to test line 260 dict structure
    fake_wrap = ["elbow_torus_wrap", 0, np.zeros(3), np.array([0.03, 0.05, 0.0]), np.eye(3), "SPHERE", [0, np.zeros(3)]]
    with patch(
        "myoconverter.conversion_steps.O2MStep2.sortMuscleWrapSiteJoint",
        side_effect=lambda m, p: {**p, "wrapping_info": [fake_wrap]},
    ):
        out_cvt2 = ma_opt.optMomentArms()

    assert os.path.isfile(out_cvt2)
    with open(save_dir / "bic_long.pkl", "rb") as f:
        after_opt = pickle.load(f)
    assert isinstance(after_opt["opt_results"][0], dict)

    # Test compMomentArmResults withmuscle_list=None (scans *.pkl, must ignore config.pkl) and zero cost_opt
    after_opt["opt_results"][0] = {"cost_org": 0.0, "cost_opt": 0.0, "par_opt": []}
    with open(save_dir / "bic_long.pkl", "wb") as f:
        pickle.dump(after_opt, f)

    ma_opt.muscle_list = None
    ma_opt.compMomentArmResults(mjc_model_path=out_cvt2)

    with open(save_dir / "bic_long.pkl", "rb") as f:
        reloaded = pickle.load(f)
    assert "mjc_ma_data" in reloaded

    with open(save_dir / "overall_comp_momentarms.pkl", "rb") as f:
        rms = pickle.load(f)
    assert rms["cost_opt_mat_mean"] == 0.0
    assert rms["cost_opt_mat_std"] == 0.0


def test_step3_skip_opt_constant_length_and_multi_activation_plot(tmp_path):
    """MuscleForceOpt must handle skipped PSO (res_opt=None), constant lengths, and multi-act passive plotting."""
    cvt2_path = tmp_path / "test_cvt2.xml"
    cvt2_path.write_text(SYNTHETIC_MJC_XML, encoding="utf-8")
    save_dir = tmp_path / "Step3_muscleKinetics"
    save_dir.mkdir()

    with open(save_dir / "config.pkl", "wb") as f:
        pickle.dump({"cfg": 1}, f)

    model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)
    jnt_sets = [(0.1,), (0.5,), (1.0,)]

    # Pre-compute matching force when gainprm[2]=gainprm[3]=1 and constant osim length to exercise both paths
    const_lengths = np.array([0.32, 0.32, 0.32])
    mus_pkl = {
        "muscle_name": "bic_long",
        "mtu_par_set": {"fiber_opt": 0.12, "tendon_sla": 0.20, "fmax": 200.0, "penn_ang": 0.0, "vmax": 10.0},
        "mtu_length_osim": const_lengths.copy(),
        "jit_list_set": jnt_sets,
        "mjc_jit_list_set": jnt_sets,
        "jit_uniq": ["elbow_flex"],
        "jit_uniq_ranges": [np.array([0.0, 2.0])],
        "act_list": [0.5, 1.0],
        "mtu_force_osim": np.zeros((2, 3)),
        "act_force_osim": np.zeros((2, 3)),
        "pas_force_osim": np.zeros((2, 3)),
    }
    with open(save_dir / "bic_long.pkl", "wb") as f:
        pickle.dump(mus_pkl, f)

    mock_osim = _make_mock_osim_model(("bic_long",))
    with patch("myoconverter.conversion_steps.O2MStep3.opensim.Model", return_value=mock_osim):
        mf_opt = MuscleForceOpt(str(cvt2_path), "dummy.osim", str(save_dir), muscle_list=["bic_long"], osim_data_overwrite=False)

    # Force getMuscleForceDiff to report below-threshold error (err_ind = []) so res_opt = None branch runs
    with patch(
        "myoconverter.conversion_steps.O2MStep3.getMuscleForceDiff",
        return_value=([], np.array([[0.30, 0.32, 0.34], [0.30, 0.32, 0.34]]), 0.001),
    ):
        out_cvt3 = mf_opt.optMuscleForce()

    assert os.path.isfile(out_cvt3)
    with open(save_dir / "bic_long.pkl", "rb") as f:
        saved = pickle.load(f)
    # Original mtu_length_osim must not have been mutated in-place
    assert np.allclose(saved["mtu_length_osim"], const_lengths)
    assert saved["opt_results"]["res_opt"] is None

    # Validate compMuscleForceResults with act_list=[0.5, 1.0] (2 activations vs 1 passive row) and muscle_list=None
    mf_opt.muscle_list = None
    mf_opt.compMuscleForceResults(mjc_model_path=out_cvt3)

    with open(save_dir / "bic_long.pkl", "rb") as f:
        reloaded = pickle.load(f)
    assert "mtu_force_mjc" in reloaded
    assert "mtu_force_mjc_passive" in reloaded


def test_sparse_actuator_moment_matches_finite_difference():
    """_get_actuator_moment must reconstruct dense rows from genuinely sparse MuJoCo 3+/4+ CSR arrays."""
    sparse_xml = """\
    <mujoco model="sparse_moment_test">
      <worldbody>
        <body name="b1">
          <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
          <joint name="j0" type="hinge" axis="1 0 0"/>
          <site name="m1_orig" pos="0 0.05 0"/>
          <geom name="cyl" type="cylinder" euler="0 1.5708 0" pos="0 0 -0.2" size="0.03 0.05" contype="0" conaffinity="0"/>
          <body name="b2" pos="0 0 -0.2">
            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
            <joint name="j1" type="hinge" axis="1 0 0"/>
            <site name="m1_ins" pos="0 0.04 -0.15"/>
            <site name="m2_orig" pos="0 -0.04 -0.05"/>
            <body name="b3" pos="0 0 -0.2">
              <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
              <joint name="j2" type="hinge" axis="1 0 0"/>
              <site name="m2_ins" pos="0 -0.03 -0.12"/>
            </body>
          </body>
        </body>
      </worldbody>
      <tendon>
        <spatial name="t1"><site site="m1_orig"/><geom geom="cyl"/><site site="m1_ins"/></spatial>
        <spatial name="t2"><site site="m2_orig"/><site site="m2_ins"/></spatial>
      </tendon>
      <actuator>
        <muscle name="m1" tendon="t1"/>
        <muscle name="m2" tendon="t2"/>
      </actuator>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(sparse_xml)
    data = mujoco.MjData(model)
    q_base = np.array([0.1, 0.4, -0.3])
    data.qpos[:] = q_base
    mujoco.mj_forward(model, data)

    # Confirm MuJoCo 3+/4+ stores fewer non-zeros than nu * nv = 6
    if data.actuator_moment.ndim == 1:
        assert int(np.sum(data.moment_rownnz)) < model.nu * model.nv

    eps = 1e-6
    for act_id in range(model.nu):
        analytic_row = _get_actuator_moment(model, data, act_id, [0, 1, 2])
        fd_row = np.zeros(model.nv)
        for dof in range(model.nv):
            data.qpos[:] = q_base
            data.qpos[dof] += eps
            mujoco.mj_forward(model, data)
            l_plus = float(data.actuator_length[act_id])

            data.qpos[:] = q_base
            data.qpos[dof] -= eps
            mujoco.mj_forward(model, data)
            l_minus = float(data.actuator_length[act_id])
            fd_row[dof] = (l_plus - l_minus) / (2.0 * eps)

        assert np.allclose(analytic_row, fd_row, rtol=1e-4, atol=1e-6)


def test_step3_six_param_muscle_force_opt_and_compliant_tendon_fit(tmp_path):
    """Issue #45: 6-parameter [range_min, range_max, lmin, lmax, fpmax, fmax] optimization & [0.5, 1.6] defaults."""
    model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)

    # 4-parameter legacy compatibility
    updateMuscleForceProperties(model, "bic_long", [0.6, 1.2, 1.4, 1.1])
    act = model.actuator("bic_long")
    assert np.allclose(act.gainprm[[0, 1, 7, 2]], [0.6, 1.2, 1.4, 1.1])
    assert np.allclose(act.biasprm[[0, 1, 7, 2]], [0.6, 1.2, 1.4, 1.1])

    # 6-parameter compliant-tendon update
    updateMuscleForceProperties(model, "bic_long", [0.45, 0.92, 0.55, 1.45, 1.8, 0.95])
    assert np.allclose(act.gainprm[[0, 1, 4, 5, 7, 2]], [0.45, 0.92, 0.55, 1.45, 1.8, 0.95])
    assert np.allclose(act.biasprm[[0, 1, 4, 5, 7, 2]], [0.45, 0.92, 0.55, 1.45, 1.8, 0.95])

    with pytest.raises(ValueError):
        updateMuscleForceProperties(model, "bic_long", [0.5, 1.0, 1.2])

    # Degenerate range_max <= range_min + 0.02 returns smooth penalty without calling simulation
    initializeWorker(model)
    pen = objFMMuscle([0.9, 0.85, 0.5, 1.6, 1.3, 1.0], np.zeros((1, 3)), "bic_long", ["elbow_flex"], [(0.2,), (0.8,), (1.4,)], [1.0])
    assert pen > 1e3

    # End-to-end Step 3 check with reversed/extensor mtu_length_osim and sub-1.0 range_max target
    cvt2_path = tmp_path / "compliant_cvt2.xml"
    cvt2_path.write_text(SYNTHETIC_MJC_XML, encoding="utf-8")
    save_dir = tmp_path / "Step3_compliant"
    save_dir.mkdir()

    jnt_sets = [(0.1,), (0.6,), (1.2,), (1.8,)]
    target_model = mujoco.MjModel.from_xml_string(SYNTHETIC_MJC_XML)
    target_model.actuator("bic_long").gainprm[3] = 1.0
    target_model.actuator("bic_long").biasprm[3] = 1.0
    # Target compliant-tendon curve has range_max = 0.88 < 1.0 (impossible under old lb[1]=1.0, lmin=0, lmax=2)
    updateMuscleForceProperties(target_model, "bic_long", [0.42, 0.88, 0.58, 1.52, 1.5, 1.0])
    f_target, l_target = getMuscleForceLengthCurvesSim(target_model, "bic_long", ["elbow_flex"], jnt_sets, [1.0])

    # Pass descending lengths to verify np.min/np.max handling on extensor-ordered arrays
    descending_len = l_target.flatten()[::-1].copy()
    mus_pkl = {
        "muscle_name": "bic_long",
        "mtu_par_set": {"fiber_opt": 0.08, "tendon_sla": 0.27, "fmax": 200.0, "penn_ang": 0.0, "vmax": 10.0},
        "mtu_length_osim": descending_len,
        "jit_list_set": jnt_sets,
        "mjc_jit_list_set": jnt_sets,
        "jit_uniq": ["elbow_flex"],
        "jit_uniq_ranges": [np.array([0.0, 2.0])],
        "act_list": [1.0],
        "mtu_force_osim": -f_target * 200.0,
        "act_force_osim": -f_target * 200.0,
        "pas_force_osim": np.zeros_like(f_target),
    }
    with open(save_dir / "bic_long.pkl", "wb") as f:
        pickle.dump(mus_pkl, f)

    mock_osim = _make_mock_osim_model(("bic_long",))
    with patch("myoconverter.conversion_steps.O2MStep3.opensim.Model", return_value=mock_osim):
        mf_opt = MuscleForceOpt(str(cvt2_path), "dummy.osim", str(save_dir), muscle_list=["bic_long"], osim_data_overwrite=False)

    # Use speedy=True via wrapper so test completes quickly in unit test suite
    orig_pso = fmOptPSO_cust
    with patch(
        "myoconverter.conversion_steps.O2MStep3.fmOptPSO_cust",
        side_effect=lambda *args, **kwargs: orig_pso(*args, speedy=True),
    ):
        out_cvt3 = mf_opt.optMuscleForce()

    opt_model = mujoco.MjModel.from_xml_path(out_cvt3)
    opt_act = opt_model.actuator("bic_long")
    assert opt_act.lengthrange[0] < opt_act.lengthrange[1]
    assert 0.2 <= opt_act.gainprm[4] <= 0.95
    assert 1.05 <= opt_act.gainprm[5] <= 1.99
    with open(save_dir / "bic_long.pkl", "rb") as f:
        opt_saved = pickle.load(f)
    assert len(opt_saved["opt_results"]["res_opt"]) == 6
    assert opt_saved["opt_results"]["cost_opt"] <= opt_saved["opt_results"]["cost_org"] + 1e-9


def test_generate_pdf_readonly_pickles_and_zero_endpoints(tmp_path):
    """generate_pdf must open pickles read-only ('rb') and handle 0 or 1 endpoint SVG files cleanly."""
    import stat
    from myoconverter.utils.generate_pdf import generate_pdf

    vlt1 = tmp_path / "vlt1"
    vlt2 = tmp_path / "vlt2"
    vlt3 = tmp_path / "vlt3"
    (vlt1 / "end_points").mkdir(parents=True)
    (vlt2 / "moment_arms").mkdir(parents=True)
    (vlt3 / "muscle_forces").mkdir(parents=True)

    p1 = vlt1 / "end_points" / "end_point_error.pkl"
    p2 = vlt2 / "overall_comp_momentarms.pkl"
    p3 = vlt3 / "overall_comp_muscleforces.pkl"
    with open(p1, "wb") as f:
        pickle.dump({"mean": 0.0, "std": 0.0}, f)
    with open(p2, "wb") as f:
        pickle.dump({"cost_opt_mat_mean": 0.001, "cost_opt_mat_std": 0.0002}, f)
    with open(p3, "wb") as f:
        pickle.dump({"rms_opt_mean": 0.02, "rms_opt_std": 0.005}, f)

    # Mark all 3 pickles read-only to verify 'rb' instead of 'rb+'
    for p in (p1, p2, p3):
        os.chmod(p, stat.S_IREAD)

    try:
        # Case 1: 0 SVG files in vlt1/end_points
        pdf0 = generate_pdf(str(vlt1), str(vlt2), str(vlt3), "test_model", str(tmp_path))
        out0 = tmp_path / "report0.pdf"
        pdf0.render(str(out0))
        assert out0.is_file() and out0.stat().st_size > 0
    finally:
        for p in (p1, p2, p3):
            os.chmod(p, stat.S_IWRITE | stat.S_IREAD)


def test_preconverted_gait10dof18musc_models_load_and_extend_knee():
    """Issues #30, #31, #41: pre-converted Gait10dof18musc cvt2 & cvt3 load in MuJoCo 4 and extend knee cleanly."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    for fname in ("gait10dof18musc_cvt2.xml", "gait10dof18musc_cvt3.xml"):
        xml_path = os.path.join(repo_root, "models", "mjc", "Gait10dof18musc", fname)
        model = mujoco.MjModel.from_xml_path(xml_path)
        assert model.opt.timestep == pytest.approx(0.002)

        data = mujoco.MjData(model)
        # Lock pelvis in air so legs swing freely without ground contact interference
        pelvis_ty_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pelvis_ty")
        knee_r_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "knee_angle_r")
        vasti_r_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "vasti_r")

        mujoco.mj_resetDataKeyframe(model, data, 0)
        data.qpos[model.jnt_qposadr[pelvis_ty_id]] = 1.3
        # Start right knee flexed at -0.8 rad and set coupled knee translations + path points via dependencyJointAng
        free_ids = [knee_r_id]
        data.qpos[model.jnt_qposadr[knee_r_id]] = -0.8
        dep_jnts, dep_angs, _ = dependencyJointAng(model, free_ids, [-0.8])
        for jid, ang in zip(dep_jnts, dep_angs):
            data.qpos[model.jnt_qposadr[jid]] = ang

        data.ctrl[vasti_r_id] = 1.0
        act_adr = int(model.actuator_actadr[vasti_r_id])
        if act_adr >= 0:
            data.act[act_adr] = 1.0

        mujoco.mj_forward(model, data)
        # Check muscle torque on knee_angle_r is positive (knee extension corresponds to increasing knee_angle_r -> 0)
        knee_dof = int(model.jnt_dofadr[knee_r_id])
        ma_vasti = _get_actuator_moment(model, data, vasti_r_id, knee_dof)
        # In MuJoCo, actuator_force < 0 (pulling), so extension torque > 0 requires moment arm < 0 (or qfrc_actuator > 0)
        assert data.qfrc_actuator[knee_dof] > 0.0, f"{fname}: vasti_r failed to produce extension torque ({ma_vasti=})"

        for _ in range(100):
            data.ctrl[vasti_r_id] = 1.0
            mujoco.mj_step(model, data)

        assert int(data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number) == 0, f"{fname} emitted mjWARN_BADQACC"
        assert data.qpos[model.jnt_qposadr[knee_r_id]] > -0.8, f"{fname}: vasti_r flexed instead of extending knee"
