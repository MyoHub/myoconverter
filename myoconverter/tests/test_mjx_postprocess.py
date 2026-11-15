"""Tests for MJX / GPU-RL post-processing of converted MuJoCo XML models."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest
from myoconverter.utils import mjx_postprocess
from myoconverter.utils.mjx_postprocess import make_mjx_ready

REPO_ROOT = Path(__file__).resolve().parents[2]
ARM26_CVT3_XML = REPO_ROOT / "models" / "mjc" / "Arm26" / "arm26_cvt3.xml"


def test_make_mjx_ready_converts_ellipsoid_and_torus_wraps_and_steps_cleanly(tmp_path):
    """Verify ellipsoid->sphere/cylinder & torus->sphere wrap conversion, joint floors, and mj_step."""
    raw_mjcf = tmp_path / "raw_ellipsoid_wrap.xml"
    raw_mjcf.write_text(
        """<mujoco model="mjx_wrap_test">
  <option timestep="0.002" collision="predefined"/>
  <default>
    <joint limited="true" damping="0.001" armature="0.00001"/>
  </default>
  <worldbody>
    <body name="pelvis" pos="0 0 1.0">
      <inertial pos="0 0 0" mass="5.0" diaginertia="0.05 0.05 0.05"/>
      <joint name="root_free" type="free"/>
      <body name="thigh" pos="0 0 -0.1">
        <inertial pos="0 0 -0.2" mass="4.0" diaginertia="0.04 0.04 0.008"/>
        <joint name="hip_flex" type="hinge" axis="0 1 0" range="-1.0 1.5" damping="0.01" armature="0.0001"/>
        <site name="m_orig" pos="0.05 0 0"/>
        <geom name="iso_wrap" type="ellipsoid" pos="0.01 0 -0.15" size="0.030 0.032 0.029" group="2"/>
        <site name="iso_side" pos="0.035 0 -0.15"/>
        <site name="m_mid" pos="0.03 0 -0.25"/>
        <geom name="elong_wrap" type="ellipsoid" pos="0.01 0 -0.35" size="0.07 0.02 0.02" group="2"/>
        <site name="elong_side" pos="0.01 0.03 -0.35"/>
        <geom name="ankle_torus" type="torus" pos="0 0 -0.40" size="0.025 0.05" group="2"/>
        <body name="shank" pos="0 0 -0.42">
          <inertial pos="0 0 -0.18" mass="2.5" diaginertia="0.02 0.02 0.004"/>
          <joint name="knee_flex" type="hinge" axis="0 1 0" range="-2.0 0.1"/>
          <site name="m_ins" pos="0.03 0 -0.1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="quad_tendon">
      <site site="m_orig"/>
      <geom geom="iso_wrap" sidesite="iso_side"/>
      <site site="m_mid"/>
      <geom geom="elong_wrap" sidesite="elong_side"/>
      <site site="m_ins"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="rect_fem" tendon="quad_tendon" lengthrange="0.35 0.60" force="600"/>
  </actuator>
</mujoco>
""",
        encoding="utf-8",
    )

    out_mjx = tmp_path / "ready_mjx.xml"
    report = make_mjx_ready(
        raw_mjcf,
        output_xml=out_mjx,
        min_damping=0.06,
        min_armature=0.008,
        integrator="implicitfast",
        iterations=15,
        ls_iterations=25,
    )

    assert report["mjx_valid"] is True
    assert Path(report["output_xml"]) == out_mjx
    assert report["solver_options"]["integrator"] == "implicitfast"
    assert report["solver_options"]["iterations"] == 15
    assert report["solver_options"]["ls_iterations"] == 25

    approx_by_geom = {entry["geom"]: entry for entry in report["wrap_approximations"]}
    assert set(approx_by_geom.keys()) == {"iso_wrap", "elong_wrap", "ankle_torus"}
    assert approx_by_geom["iso_wrap"]["replacement_type"] == "sphere"
    assert approx_by_geom["elong_wrap"]["replacement_type"] == "cylinder"
    assert approx_by_geom["elong_wrap"]["replacement_size"] == pytest.approx(
        [0.02, 0.07], rel=1e-4
    )
    assert approx_by_geom["ankle_torus"]["replacement_type"] == "sphere"

    model = mujoco.MjModel.from_xml_path(str(out_mjx))
    assert model.opt.integrator == int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
    assert model.opt.iterations == 15
    assert model.opt.ls_iterations == 25

    # Free joint (first 6 DoFs) untouched; hinge joints floored to >= 0.06 damping and >= 0.008 armature
    assert np.all(model.dof_damping[:6] == 0.0)
    assert np.all(model.dof_armature[:6] == 0.0)
    assert np.all(model.dof_damping[6:] >= 0.06 - 1e-9)
    assert np.all(model.dof_armature[6:] >= 0.008 - 1e-9)

    data = mujoco.MjData(model)
    data.ctrl[:] = 0.5
    for _ in range(25):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_make_mjx_ready_bundled_arm26_and_cli_report(tmp_path):
    """Verify bundled Arm26 model post-processing and standalone CLI report output."""
    out_xml = tmp_path / "arm26_mjx.xml"
    report_json = tmp_path / "arm26_mjx_report.json"

    mjx_postprocess.main(
        [
            str(ARM26_CVT3_XML),
            "--output_xml",
            str(out_xml),
            "--integrator",
            "Euler",
            "--report_json",
            str(report_json),
        ]
    )
    assert out_xml.is_file()
    assert report_json.is_file()
    payload = json.loads(report_json.read_text(encoding="utf-8"))
    assert payload["mjx_valid"] is True
    assert payload["solver_options"]["integrator"] == "Euler"
    assert payload["model_stats"]["nu"] == 6


def test_make_mjx_ready_validates_inputs(tmp_path):
    """Verify validation errors on missing file, negative floors, or invalid integrator."""
    with pytest.raises(FileNotFoundError):
        make_mjx_ready(tmp_path / "missing.xml")

    xml_path = tmp_path / "minimal.xml"
    xml_path.write_text("<mujoco><worldbody/></mujoco>", encoding="utf-8")
    with pytest.raises(ValueError, match="non-negative"):
        make_mjx_ready(xml_path, min_damping=-0.1)
    with pytest.raises(ValueError, match="Unsupported integrator"):
        make_mjx_ready(xml_path, integrator="BadSolver")
