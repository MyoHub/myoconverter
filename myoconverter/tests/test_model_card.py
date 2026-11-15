"""Tests for JSON conversion and validation model card exporter."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from myoconverter.utils import model_card
from myoconverter.utils.model_card import export_model_card, load_mjmodel_compat

REPO_ROOT = Path(__file__).resolve().parents[2]
ARM26_MJC_DIR = REPO_ROOT / "models" / "mjc" / "Arm26"
ARM26_OSIM_FILE = REPO_ROOT / "models" / "osim" / "Arm26" / "arm26.osim"


def test_export_model_card_with_arm26_validation_artifacts_and_quality_gate(tmp_path):
    """Verify JSON model card schema, SHA-256 provenance, model summary, and Step 1-3 RMSEs."""
    compiled = load_mjmodel_compat(ARM26_MJC_DIR / "arm26_cvt3.xml")
    assert compiled.nq == 2

    card_path = export_model_card(
        output_folder=tmp_path,
        model_name="arm26",
        osim_file=ARM26_OSIM_FILE,
        mjcf_file=ARM26_MJC_DIR / "arm26_cvt3.xml",
        vlt1_data=ARM26_MJC_DIR / "Step1_xmlConvert",
        vlt2_data=ARM26_MJC_DIR / "Step2_muscleKinematics",
        vlt3_data=ARM26_MJC_DIR / "Step3_muscleKinetics",
        convert_steps=[1, 2, 3],
    )
    assert card_path == tmp_path / "arm26_model_card.json"
    assert card_path.is_file()

    card = json.loads(card_path.read_text(encoding="utf-8"))
    assert card["schema_version"] == "1.0.0"
    assert card["model_name"] == "arm26"

    prov = card["provenance"]
    assert len(prov["osim_sha256"]) == 64
    assert len(prov["mjcf_sha256"]) == 64
    assert prov["conversion_steps_run"] == [1, 2, 3]

    summary = card["model_summary"]
    assert summary["compiled"] is True
    assert summary["nq"] == 2
    assert summary["nv"] == 2
    assert summary["nu"] == 6
    assert summary["ntendon"] == 6
    assert set(summary["joint_names"]) == {"r_shoulder_elev", "r_elbow_flex"}
    assert len(summary["muscle_names"]) == 6
    assert summary["muscle_peak_forces"]["TRIlong"] == pytest.approx(762.765, rel=1e-4)

    val = card["validation"]
    assert "r_shoulder_elev" in val["joint_rom_rad"]
    assert "r_elbow_flex" in val["joint_rom_rad"]
    assert val["kinematic_rmse"] == pytest.approx(8.597596e-05, rel=1e-3)
    assert len(val["moment_arm_rmse_per_muscle"]) == 6
    assert val["moment_arm_rmse_max"] == pytest.approx(0.00248878, rel=1e-3)
    assert len(val["muscle_force_rmse_per_muscle"]) == 6
    assert val["muscle_force_rmse_max"] == pytest.approx(0.1951648, rel=1e-3)
    assert val["passed_quality_gate"] is True

    # Synthetic out-of-tolerance validation metrics fail the quality gate
    bad_dir = tmp_path / "degraded"
    bad_card_path = export_model_card(
        output_folder=bad_dir,
        model_name="bad_arm",
        mjcf_file=ARM26_MJC_DIR / "arm26_cvt3.xml",
        vlt1_data={
            "kinematic_rmse": 0.08,
            "joint_rom_rad": {"r_elbow_flex": [0.0, 2.2]},
        },
        vlt2_data={"moment_arm_rmse_per_muscle": {"BIClong": 0.03}},
        vlt3_data={"muscle_force_rmse_per_muscle": {"BIClong": 0.55}},
    )
    bad_card = json.loads(bad_card_path.read_text(encoding="utf-8"))
    assert bad_card["validation"]["passed_quality_gate"] is False
    assert bad_card["validation"]["moment_arm_rmse_max"] == pytest.approx(0.03)
    assert bad_card["validation"]["muscle_force_rmse_max"] == pytest.approx(0.55)


def test_model_card_cli_entrypoint(tmp_path):
    """Verify standalone CLI entrypoint writes `<model_name>_model_card.json`."""
    cli_out = tmp_path / "cli_out"
    out_path = model_card.main(
        [
            str(cli_out),
            "arm26_cli",
            "--osim_file",
            str(ARM26_OSIM_FILE),
            "--mjcf_file",
            str(ARM26_MJC_DIR / "arm26_cvt3.xml"),
        ]
    )
    assert out_path == cli_out / "arm26_cli_model_card.json"
    assert out_path.is_file()
