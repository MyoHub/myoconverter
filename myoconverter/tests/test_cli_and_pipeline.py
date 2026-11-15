"""Regression tests for CLI argument parsing, scoped loguru handlers, and CI/conda config."""

import argparse
from pathlib import Path
import sys
import types
from unittest.mock import MagicMock, patch

from loguru import logger
import pytest

# Allow importing O2MPipeline even when OpenSim is not installed in a lightweight test venv.
for _mod_name in ("opensim",):
    if _mod_name not in sys.modules:
        try:
            __import__(_mod_name)
        except ModuleNotFoundError:
            _stub = types.ModuleType(_mod_name)
            _stub.Model = MagicMock
            _stub.CoordinateCouplerConstraint = MagicMock()
            sys.modules[_mod_name] = _stub

from myoconverter.O2MPipeline import (
    O2MPipeline,
    parse_convert_steps,
    parse_muscle_list,
    parse_pipeline_args,
    str2bool,
)
import examples.cli as example_cli


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_str2bool_parses_truthy_falsy_and_rejects_invalid():
    assert str2bool(True) is True
    assert str2bool(False) is False
    for val in ("true", "True", "TRUE", "1", "yes", "y", "t"):
        assert str2bool(val) is True
    for val in ("false", "False", "FALSE", "0", "no", "n", "f"):
        assert str2bool(val) is False
    with pytest.raises(argparse.ArgumentTypeError):
        str2bool("maybe")


def test_parse_convert_steps_and_muscle_list_formats():
    assert parse_convert_steps(["1", "2", "3"]) == [1, 2, 3]
    assert parse_convert_steps(["1,3"]) == [1, 3]
    assert parse_convert_steps(["[2, 3]"]) == [2, 3]

    with pytest.raises(argparse.ArgumentTypeError):
        parse_convert_steps(["4"])
    with pytest.raises(argparse.ArgumentTypeError):
        parse_convert_steps(["abc"])

    assert parse_muscle_list(None) is None
    assert parse_muscle_list(["bic_long", "tric_long"]) == ["bic_long", "tric_long"]
    assert parse_muscle_list(["bic_long, tric_long"]) == ["bic_long", "tric_long"]


def test_o2mpipeline_cli_argparser_handles_booleans_and_lists():
    args = parse_pipeline_args(["model.osim", "geom_dir", "out_dir"])
    assert args.convert_steps == [1, 2, 3]
    assert args.muscle_list is None
    assert args.conversion is True
    assert args.validation is True
    assert args.speedy is False

    args_custom = parse_pipeline_args(
        [
            "model.osim",
            "geom_dir",
            "out_dir",
            "--convert_steps",
            "1",
            "3",
            "--muscle_list",
            "bic_l",
            "tric_l",
            "--speedy",
            "--conversion",
            "False",
            "--validation",
            "0",
            "--add_ground_geom",
            "True",
        ]
    )
    assert args_custom.convert_steps == [1, 3]
    assert args_custom.muscle_list == ["bic_l", "tric_l"]
    assert args_custom.speedy is True
    assert args_custom.conversion is False
    assert args_custom.validation is False
    assert args_custom.add_ground_geom is True


def test_examples_cli_forwards_parsed_types_to_pipeline():
    with patch.object(example_cli, "O2MPipeline") as mock_pipeline:
        example_cli.main(
            [
                "--osim_file",
                "arm26.osim",
                "--geometry_folder",
                "Geometry",
                "--output_folder",
                "out",
                "--convert_steps",
                "1",
                "2",
                "--muscle_list",
                "TRIlong",
                "BIClong",
                "--speedy",
                "False",
                "--generate_pdf",
                "False",
            ]
        )

    mock_pipeline.assert_called_once()
    kwargs = mock_pipeline.call_args.kwargs
    assert kwargs["convert_steps"] == [1, 2]
    assert kwargs["muscle_list"] == ["TRIlong", "BIClong"]
    assert kwargs["speedy"] is False
    assert kwargs["generate_pdf"] is False


def test_o2mpipeline_preserves_caller_loguru_handlers(tmp_path):
    captured = []
    sink_id = logger.add(lambda msg: captured.append(str(msg)))
    try:
        with patch("myoconverter.O2MPipeline.O2MSteps") as mock_steps_cls:
            O2MPipeline(
                str(tmp_path / "arm26.osim"),
                str(tmp_path / "Geometry"),
                str(tmp_path / "out"),
                convert_steps=[1],
            )
            mock_steps_cls.return_value.PipelineExecution.assert_called_once()

        log_file = tmp_path / "out" / "arm26_conversion.log"
        assert log_file.is_file()

        # Caller's sink must still be active after O2MPipeline exits!
        captured.clear()
        logger.info("post-pipeline caller log check")
        assert any("post-pipeline caller log check" in entry for entry in captured)

        # Even if PipelineExecution raises, scoped log file handler is cleaned up and caller sink survives
        with patch("myoconverter.O2MPipeline.O2MSteps") as mock_steps_err:
            mock_steps_err.return_value.PipelineExecution.side_effect = RuntimeError("boom")
            with pytest.raises(RuntimeError, match="boom"):
                O2MPipeline(
                    str(tmp_path / "arm26.osim"),
                    str(tmp_path / "Geometry"),
                    str(tmp_path / "out_err"),
                    convert_steps=[1],
                )

        captured.clear()
        logger.info("after-error caller log check")
        assert any("after-error caller log check" in entry for entry in captured)
    finally:
        logger.remove(sink_id)


def test_conda_env_and_ci_workflow_configuration():
    conda_text = (REPO_ROOT / "conda_env.yml").read_text(encoding="utf-8")
    assert "- networkx" in conda_text
    assert "- scipy" in conda_text
    assert "- opensim>=4.0" in conda_text
    assert "opensim=>4.0" not in conda_text

    workflow_text = (REPO_ROOT / ".github" / "workflows" / "python-app.yml").read_text(encoding="utf-8")
    assert '" feature/*"' not in workflow_text
    assert '"development"' in workflow_text
    assert '"docs/**"' in workflow_text
    assert "env create -n myoconverter -f conda_env.yml" in workflow_text
    assert "installation-dir: ${{ runner.temp }}/miniforge3" in workflow_text
    assert "conda config --show-sources" in workflow_text
    assert "${{ matrix.conda-cmd }} config --show-sources" not in workflow_text
