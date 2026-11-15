"""Tests for OpenSim ``<ScaleSet>`` and anthropometric MJCF scaling."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest
from lxml import etree
from myoconverter.utils import mjcf_scaling
from myoconverter.utils.mjcf_scaling import (
    _sanitize_mjcf_tree_for_mujoco3,
    parse_opensim_scaleset,
    scale_converted_mjcf,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
ARM26_CVT3_XML = REPO_ROOT / "models" / "mjc" / "Arm26" / "arm26_cvt3.xml"


def _load_arm26_compat() -> mujoco.MjModel:
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(str(ARM26_CVT3_XML), parser)
    root = _sanitize_mjcf_tree_for_mujoco3(tree, base_dir=ARM26_CVT3_XML.parent)
    return mujoco.MjModel.from_xml_string(etree.tostring(root, encoding="unicode"))


def test_scale_converted_mjcf_from_opensim_scaleset_xml(tmp_path):
    """Verify zero-OpenSim ``<ScaleSet>`` XML parsing and subject-specific geometric/mass/tendon scaling."""
    scaleset_xml = tmp_path / "subject_scaleset.xml"
    scaleset_xml.write_text(
        """<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
  <ScaleSet name="subject01_scales">
    <objects>
      <Scale name="humerus_scale">
        <scales>1.10 1.15 1.10</scales>
        <segment>r_humerus</segment>
        <apply>true</apply>
      </Scale>
      <Scale name="forearm_scale">
        <scales>1.05 1.12 1.05</scales>
        <segment>r_ulna_radius_hand</segment>
        <apply>true</apply>
      </Scale>
      <Scale name="disabled_scale">
        <scales>9.99 9.99 9.99</scales>
        <segment>base</segment>
        <apply>false</apply>
      </Scale>
    </objects>
  </ScaleSet>
</OpenSimDocument>
""",
        encoding="utf-8",
    )

    parsed = parse_opensim_scaleset(scaleset_xml)
    assert set(parsed.keys()) == {"r_humerus", "r_ulna_radius_hand"}
    assert parsed["r_humerus"] == pytest.approx((1.10, 1.15, 1.10))

    orig_model = _load_arm26_compat()
    scaled_xml = scale_converted_mjcf(
        ARM26_CVT3_XML,
        output_xml=tmp_path / "arm26_subject01.xml",
        scale_set_xml=scaleset_xml,
    )
    assert scaled_xml.is_file()

    scaled_model = mujoco.MjModel.from_xml_path(str(scaled_xml))
    hum_id = mujoco.mj_name2id(scaled_model, mujoco.mjtObj.mjOBJ_BODY, "r_humerus")
    ulna_id = mujoco.mj_name2id(
        scaled_model, mujoco.mjtObj.mjOBJ_BODY, "r_ulna_radius_hand"
    )

    # Volumetric mass increase: r_humerus *= 1.10 * 1.15 * 1.10 = 1.3915
    assert scaled_model.body_mass[hum_id] == pytest.approx(
        orig_model.body_mass[hum_id] * 1.3915, rel=1e-4
    )
    # Elbow offset on forearm is expressed in parent (r_humerus) frame -> scaled by (1.10, 1.15, 1.10)
    np.testing.assert_allclose(
        scaled_model.body_pos[ulna_id],
        orig_model.body_pos[ulna_id] * np.array([1.10, 1.15, 1.10]),
        rtol=1e-4,
    )
    # Muscle lengthrange elongates along with the scaled upper-limb segments
    assert np.all(
        scaled_model.actuator_lengthrange[:, 1] > orig_model.actuator_lengthrange[:, 1]
    )


def test_scale_converted_mjcf_anthropometric_and_biomechanical_params(tmp_path):
    """Verify anthropometric mass, strength, passive joint mechanics, ROM, and CLI scaling."""
    orig_model = _load_arm26_compat()
    orig_mass = float(np.sum(orig_model.body_mass[1:]))

    subject_params = {
        "height_scale": 0.94,
        "mass_scale": 0.80,
        "strength_scale": 0.65,
        "muscle_strength_scales": {"BIClong": 0.80},
        "passive_damping_scale": 3.0,
        "passive_stiffness_scale": 2.5,
        "joint_rom_scales": {"r_elbow_flex": 0.75, "r_shoulder_elev": 0.85},
    }

    out_xml = scale_converted_mjcf(
        ARM26_CVT3_XML,
        subject_params,
        output_xml=tmp_path / "arm26_scaled_subject.xml",
    )
    scaled_model = mujoco.MjModel.from_xml_path(str(out_xml))
    scaled_data = mujoco.MjData(scaled_model)
    mujoco.mj_step(scaled_model, scaled_data)
    assert np.all(np.isfinite(scaled_data.qpos))

    assert float(np.sum(scaled_model.body_mass[1:])) == pytest.approx(
        orig_mass * 0.80, rel=1e-4
    )

    tri_id = mujoco.mj_name2id(scaled_model, mujoco.mjtObj.mjOBJ_ACTUATOR, "TRIlong")
    bic_id = mujoco.mj_name2id(scaled_model, mujoco.mjtObj.mjOBJ_ACTUATOR, "BIClong")
    assert scaled_model.actuator_gainprm[tri_id, 2] == pytest.approx(
        orig_model.actuator_gainprm[tri_id, 2] * 0.65, rel=1e-4
    )
    assert scaled_model.actuator_gainprm[bic_id, 2] == pytest.approx(
        orig_model.actuator_gainprm[bic_id, 2] * 0.52, rel=1e-4
    )
    assert scaled_model.actuator_biasprm[bic_id, 2] == pytest.approx(
        orig_model.actuator_biasprm[bic_id, 2] * 0.52, rel=1e-4
    )

    np.testing.assert_allclose(
        scaled_model.dof_damping, orig_model.dof_damping * 3.0, rtol=1e-4
    )

    elb_id = mujoco.mj_name2id(scaled_model, mujoco.mjtObj.mjOBJ_JOINT, "r_elbow_flex")
    sh_id = mujoco.mj_name2id(
        scaled_model, mujoco.mjtObj.mjOBJ_JOINT, "r_shoulder_elev"
    )
    orig_elb_span = orig_model.jnt_range[elb_id, 1] - orig_model.jnt_range[elb_id, 0]
    new_elb_span = (
        scaled_model.jnt_range[elb_id, 1] - scaled_model.jnt_range[elb_id, 0]
    )
    assert new_elb_span == pytest.approx(orig_elb_span * 0.75, rel=1e-4)

    orig_sh_span = orig_model.jnt_range[sh_id, 1] - orig_model.jnt_range[sh_id, 0]
    new_sh_span = scaled_model.jnt_range[sh_id, 1] - scaled_model.jnt_range[sh_id, 0]
    assert new_sh_span == pytest.approx(orig_sh_span * 0.85, rel=1e-4)

    cli_xml = tmp_path / "arm26_cli_scaled.xml"
    mjcf_scaling.main(
        [str(ARM26_CVT3_XML), "--output_xml", str(cli_xml), "--mass_scale", "1.25"]
    )
    cli_model = mujoco.MjModel.from_xml_path(str(cli_xml))
    assert float(np.sum(cli_model.body_mass[1:])) == pytest.approx(
        orig_mass * 1.25, rel=1e-4
    )
