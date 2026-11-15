"""Machine-readable JSON model card exporter for MyoConverter conversions.

Exports ``<output_folder>/<model_name>_model_card.json`` summarizing SHA-256
provenance, compiled MuJoCo structural and actuation metadata, Step 1-3
biomechanical validation RMSE metrics, and configurable quality-gate status.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import math
import os
import pickle
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from loguru import logger
from lxml import etree

SCHEMA_VERSION = "1.0.0"

_EXCLUDED_PKL_STEMS = {
    "config",
    "overall_comp_momentarms",
    "overall_comp_muscleforces",
    "jnt_ranges_mjc",
    "jnt_ranges_osim",
    "end_point_error",
}


def _sha256_file(path: Path | None) -> str | None:
    if path is None or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _clean_float(val: Any) -> float | None:
    if val is None:
        return None
    fval = float(val)
    if not math.isfinite(fval):
        return None
    return fval


def _sanitize_mjcf_tree_for_mujoco3(
    tree: etree._ElementTree, base_dir: Path | None = None
) -> etree._Element:
    """Upgrade legacy MuJoCo 2 attributes (e.g. ``<option collision='predefined'/>``) in-memory."""
    root = tree.getroot()
    had_predefined_collision = False
    for opt in root.findall("option"):
        if opt.attrib.get("collision") == "predefined":
            had_predefined_collision = True
        opt.attrib.pop("collision", None)

    if had_predefined_collision:
        default_el = root.find("default")
        if default_el is None:
            default_el = etree.SubElement(root, "default")
        geom_def = default_el.find("geom")
        if geom_def is None:
            geom_def = etree.SubElement(default_el, "geom")
        if "contype" not in geom_def.attrib:
            geom_def.attrib["contype"] = "0"
        if "conaffinity" not in geom_def.attrib:
            geom_def.attrib["conaffinity"] = "0"

    if base_dir is not None:
        compiler_el = root.find("compiler")
        if compiler_el is None:
            compiler_el = etree.Element("compiler")
            root.insert(0, compiler_el)
        if "meshdir" not in compiler_el.attrib:
            compiler_el.attrib["meshdir"] = str(base_dir.resolve())
        elif not Path(compiler_el.attrib["meshdir"]).is_absolute():
            compiler_el.attrib["meshdir"] = str(
                (base_dir / compiler_el.attrib["meshdir"]).resolve()
            )

    return root


def load_mjmodel_compat(mjcf_path: Path) -> mujoco.MjModel:
    """Compile a MuJoCo XML path with automatic MuJoCo 2 -> 3 ``<option collision>`` compatibility."""
    try:
        return mujoco.MjModel.from_xml_path(str(mjcf_path))
    except ValueError as exc:
        if "collision" not in str(exc):
            raise
        parser = etree.XMLParser(remove_blank_text=True)
        tree = etree.parse(str(mjcf_path), parser)
        root = _sanitize_mjcf_tree_for_mujoco3(tree, base_dir=mjcf_path.parent)
        xml_str = etree.tostring(root, encoding="unicode")
        return mujoco.MjModel.from_xml_string(xml_str)


def _discover_mjcf_file(output_folder: Path, model_name: str) -> Path | None:
    candidates = [
        output_folder / f"{model_name}_mjx.xml",
        output_folder / f"{model_name}_scaled.xml",
        output_folder / f"{model_name}_cvt3.xml",
        output_folder / f"{model_name}_cvt2.xml",
        output_folder / f"{model_name}_cvt1.xml",
        output_folder / f"{model_name}.xml",
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    return None


def _extract_model_summary(mjcf_path: Path | None) -> dict[str, Any]:
    if mjcf_path is None or not mjcf_path.is_file():
        return {
            "compiled": False,
            "nq": 0,
            "nv": 0,
            "nu": 0,
            "nbody": 0,
            "njnt": 0,
            "ntendon": 0,
            "total_mass_kg": 0.0,
            "body_names": [],
            "joint_names": [],
            "joint_ranges": {},
            "joints": [],
            "muscle_names": [],
            "muscle_peak_forces": {},
            "actuators": [],
        }

    model = load_mjmodel_compat(mjcf_path)

    body_names: list[str] = []
    for bid in range(1, model.nbody):
        bname = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
        )
        if bname != "ground":
            body_names.append(bname)

    total_mass_kg = float(np.sum(model.body_mass[1:])) if model.nbody > 1 else 0.0

    jnt_type_map = {
        int(mujoco.mjtJoint.mjJNT_FREE): "free",
        int(mujoco.mjtJoint.mjJNT_BALL): "ball",
        int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
        int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
    }

    joint_names: list[str] = []
    joint_ranges: dict[str, list[float]] = {}
    joints_detail: list[dict[str, Any]] = []
    for jid in range(model.njnt):
        jname = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or f"joint_{jid}"
        )
        jtype_int = int(model.jnt_type[jid])
        jtype = jnt_type_map.get(jtype_int, "unknown")
        limited = bool(model.jnt_limited[jid])
        rng = [float(model.jnt_range[jid, 0]), float(model.jnt_range[jid, 1])]
        dof_adr = int(model.jnt_dofadr[jid])
        damping = float(model.dof_damping[dof_adr]) if 0 <= dof_adr < model.nv else 0.0
        armature = (
            float(model.dof_armature[dof_adr]) if 0 <= dof_adr < model.nv else 0.0
        )
        stiffness = float(model.jnt_stiffness[jid])

        joint_names.append(jname)
        if jtype in {"hinge", "slide"}:
            joint_ranges[jname] = rng
        joints_detail.append(
            {
                "name": jname,
                "type": jtype,
                "limited": limited,
                "range": rng,
                "damping": damping,
                "stiffness": stiffness,
                "armature": armature,
            }
        )

    muscle_names: list[str] = []
    muscle_peak_forces: dict[str, float] = {}
    actuators_detail: list[dict[str, Any]] = []
    muscle_gain_enum = int(mujoco.mjtGain.mjGAIN_MUSCLE)
    muscle_dyn_enum = int(mujoco.mjtDyn.mjDYN_MUSCLE)

    for aid in range(model.nu):
        aname = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            or f"actuator_{aid}"
        )
        gaintype = int(model.actuator_gaintype[aid])
        dyntype = int(model.actuator_dyntype[aid])
        gainprm = model.actuator_gainprm[aid]
        is_muscle = (gaintype == muscle_gain_enum) or (dyntype == muscle_dyn_enum)

        if is_muscle:
            peak_force = (
                float(gainprm[2]) if float(gainprm[2]) > 0.0 else float(gainprm[3])
            )
            muscle_names.append(aname)
            muscle_peak_forces[aname] = peak_force
            actuators_detail.append(
                {
                    "name": aname,
                    "type": "muscle",
                    "peak_force": peak_force,
                    "lengthrange": [
                        float(model.actuator_lengthrange[aid, 0]),
                        float(model.actuator_lengthrange[aid, 1]),
                    ],
                }
            )
        else:
            actuators_detail.append(
                {
                    "name": aname,
                    "type": "motor",
                    "peak_force": float(abs(gainprm[0])),
                }
            )

    return {
        "compiled": True,
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nbody": int(model.nbody),
        "njnt": int(model.njnt),
        "ntendon": int(model.ntendon),
        "total_mass_kg": total_mass_kg,
        "body_names": body_names,
        "joint_names": joint_names,
        "joint_ranges": joint_ranges,
        "joints": joints_detail,
        "muscle_names": muscle_names,
        "muscle_peak_forces": muscle_peak_forces,
        "actuators": actuators_detail,
    }


def _load_vlt1(output_folder: Path, vlt1_data: Any) -> dict[str, Any]:
    joint_rom_rad: dict[str, list[float]] = {}
    kinematic_rmse: float | None = None
    kinematic_rmse_std: float | None = None

    step1_dir: Path | None = None
    if vlt1_data is None:
        cand = output_folder / "Step1_xmlConvert"
        if cand.is_dir():
            step1_dir = cand
    elif isinstance(vlt1_data, (str, os.PathLike)):
        cand = Path(vlt1_data)
        if cand.is_dir():
            step1_dir = cand

    if step1_dir is not None:
        jnt_mjc_pkl = step1_dir / "jnt_ranges_mjc.pkl"
        jnt_osim_pkl = step1_dir / "jnt_ranges_osim.pkl"
        target_jnt_pkl = (
            jnt_mjc_pkl
            if jnt_mjc_pkl.is_file()
            else (jnt_osim_pkl if jnt_osim_pkl.is_file() else None)
        )
        if target_jnt_pkl is not None:
            with target_jnt_pkl.open("rb") as fp:
                raw_ranges = pickle.load(fp)
            if isinstance(raw_ranges, dict):
                for k, v in raw_ranges.items():
                    arr = np.asarray(v, dtype=float).reshape(-1)
                    if arr.size >= 2:
                        joint_rom_rad[str(k)] = [float(arr[0]), float(arr[1])]

        ep_pkl = step1_dir / "end_points" / "end_point_error.pkl"
        if ep_pkl.is_file():
            with ep_pkl.open("rb") as fp:
                ep = pickle.load(fp)
            if isinstance(ep, dict):
                kinematic_rmse = _clean_float(ep.get("mean"))
                kinematic_rmse_std = _clean_float(ep.get("std"))

    elif isinstance(vlt1_data, dict):
        if "kinematic_rmse" in vlt1_data:
            kinematic_rmse = _clean_float(vlt1_data.get("kinematic_rmse"))
            kinematic_rmse_std = _clean_float(vlt1_data.get("kinematic_rmse_std"))
        elif "end_point_error" in vlt1_data and isinstance(
            vlt1_data["end_point_error"], dict
        ):
            kinematic_rmse = _clean_float(vlt1_data["end_point_error"].get("mean"))
            kinematic_rmse_std = _clean_float(vlt1_data["end_point_error"].get("std"))
        elif "mean" in vlt1_data:
            kinematic_rmse = _clean_float(vlt1_data.get("mean"))
            kinematic_rmse_std = _clean_float(vlt1_data.get("std"))

        for rng_key in (
            "joint_rom_rad",
            "jnt_ranges_mjc",
            "jnt_ranges_osim",
            "joint_ranges",
        ):
            if rng_key in vlt1_data and isinstance(vlt1_data[rng_key], dict):
                for k, v in vlt1_data[rng_key].items():
                    arr = np.asarray(v, dtype=float).reshape(-1)
                    if arr.size >= 2:
                        joint_rom_rad[str(k)] = [float(arr[0]), float(arr[1])]
                break

    return {
        "joint_rom_rad": joint_rom_rad,
        "kinematic_rmse": kinematic_rmse,
        "kinematic_rmse_std": kinematic_rmse_std,
    }


def _extract_muscle_ma_rmse(muscle_dict: dict[str, Any]) -> float | None:
    opt_res = muscle_dict.get("opt_results")
    if isinstance(opt_res, list):
        costs = []
        for entry in opt_res:
            if isinstance(entry, dict) and "cost_opt" in entry:
                val = _clean_float(entry["cost_opt"])
                if val is not None:
                    costs.append(val)
        if costs:
            return float(max(costs))
    osim_ma = muscle_dict.get("osim_ma_data")
    mjc_ma = muscle_dict.get("mjc_ma_data")
    if (
        isinstance(osim_ma, list)
        and isinstance(mjc_ma, list)
        and osim_ma
        and len(osim_ma) == len(mjc_ma)
    ):
        errs = []
        for o_arr, m_arr in zip(osim_ma, mjc_ma):
            o_flat = np.asarray(o_arr, dtype=float).reshape(-1)
            m_flat = np.asarray(m_arr, dtype=float).reshape(-1)
            if o_flat.size > 0 and o_flat.size == m_flat.size:
                errs.append(float(np.sqrt(np.mean((o_flat + m_flat) ** 2))))
        if errs:
            return float(max(errs))
    return None


def _load_vlt2(output_folder: Path, vlt2_data: Any) -> dict[str, Any]:
    per_muscle: dict[str, float] = {}
    max_rmse: float | None = None

    step2_dir: Path | None = None
    if vlt2_data is None:
        for folder_name in (
            "Step2_muscleKinematics",
            "Step2_momentArmOpt",
            "Step2_MomentArmOpt",
        ):
            cand = output_folder / folder_name
            if cand.is_dir():
                step2_dir = cand
                break
    elif isinstance(vlt2_data, (str, os.PathLike)):
        cand = Path(vlt2_data)
        if cand.is_dir():
            step2_dir = cand

    if step2_dir is not None:
        for pkl_path_str in sorted(glob.glob(str(step2_dir / "*.pkl"))):
            pkl_path = Path(pkl_path_str)
            if pkl_path.stem in _EXCLUDED_PKL_STEMS:
                continue
            with pkl_path.open("rb") as fp:
                mdata = pickle.load(fp)
            if isinstance(mdata, dict):
                mname = str(mdata.get("muscle_name", pkl_path.stem))
                rmse = _extract_muscle_ma_rmse(mdata)
                if rmse is not None:
                    per_muscle[mname] = rmse

        overall_pkl = step2_dir / "overall_comp_momentarms.pkl"
        if overall_pkl.is_file():
            with overall_pkl.open("rb") as fp:
                ov = pickle.load(fp)
            if isinstance(ov, dict) and "cost_opt_mat" in ov:
                mat = np.asarray(ov["cost_opt_mat"], dtype=float)
                if mat.size > 0:
                    mat_max = _clean_float(np.max(mat))
                    if mat_max is not None:
                        max_rmse = mat_max

    elif isinstance(vlt2_data, dict):
        if "moment_arm_rmse_per_muscle" in vlt2_data and isinstance(
            vlt2_data["moment_arm_rmse_per_muscle"], dict
        ):
            for k, v in vlt2_data["moment_arm_rmse_per_muscle"].items():
                fv = _clean_float(v)
                if fv is not None:
                    per_muscle[str(k)] = fv
            if "moment_arm_rmse_max" in vlt2_data:
                max_rmse = _clean_float(vlt2_data["moment_arm_rmse_max"])
        elif "muscles" in vlt2_data and isinstance(vlt2_data["muscles"], dict):
            for k, v in vlt2_data["muscles"].items():
                fv = (
                    _extract_muscle_ma_rmse(v)
                    if isinstance(v, dict)
                    else _clean_float(v)
                )
                if fv is not None:
                    per_muscle[str(k)] = fv
            ov_dict = vlt2_data.get("overall")
            if isinstance(ov_dict, dict) and "cost_opt_mat" in ov_dict:
                mat = np.asarray(ov_dict["cost_opt_mat"], dtype=float)
                if mat.size > 0:
                    max_rmse = _clean_float(np.max(mat))
        elif "cost_opt_mat" in vlt2_data:
            mat = np.asarray(vlt2_data["cost_opt_mat"], dtype=float)
            if mat.size > 0:
                max_rmse = _clean_float(np.max(mat))
        else:
            for k, v in vlt2_data.items():
                if isinstance(v, dict):
                    fv = _extract_muscle_ma_rmse(v)
                elif isinstance(v, (int, float, np.floating, np.integer)):
                    fv = _clean_float(v)
                else:
                    fv = None
                if fv is not None:
                    per_muscle[str(k)] = fv

    if per_muscle:
        per_muscle_max = max(per_muscle.values())
        max_rmse = (
            per_muscle_max if max_rmse is None else max(max_rmse, per_muscle_max)
        )

    return {
        "moment_arm_rmse_per_muscle": per_muscle,
        "moment_arm_rmse_max": max_rmse,
    }


def _extract_muscle_force_rmse(muscle_dict: dict[str, Any]) -> float | None:
    opt_res = muscle_dict.get("opt_results")
    if isinstance(opt_res, dict) and "cost_opt" in opt_res:
        return _clean_float(opt_res["cost_opt"])
    return None


def _load_vlt3(output_folder: Path, vlt3_data: Any) -> dict[str, Any]:
    per_muscle: dict[str, float] = {}
    max_rmse: float | None = None

    step3_dir: Path | None = None
    if vlt3_data is None:
        for folder_name in (
            "Step3_muscleKinetics",
            "Step3_muscleForceOpt",
            "Step3_MuscleForceOpt",
        ):
            cand = output_folder / folder_name
            if cand.is_dir():
                step3_dir = cand
                break
    elif isinstance(vlt3_data, (str, os.PathLike)):
        cand = Path(vlt3_data)
        if cand.is_dir():
            step3_dir = cand

    if step3_dir is not None:
        for pkl_path_str in sorted(glob.glob(str(step3_dir / "*.pkl"))):
            pkl_path = Path(pkl_path_str)
            if pkl_path.stem in _EXCLUDED_PKL_STEMS:
                continue
            with pkl_path.open("rb") as fp:
                mdata = pickle.load(fp)
            if isinstance(mdata, dict):
                mname = str(mdata.get("muscle_name", pkl_path.stem))
                rmse = _extract_muscle_force_rmse(mdata)
                if rmse is not None:
                    per_muscle[mname] = rmse

        overall_pkl = step3_dir / "overall_comp_muscleforces.pkl"
        if overall_pkl.is_file():
            with overall_pkl.open("rb") as fp:
                ov = pickle.load(fp)
            if isinstance(ov, dict) and "rms_opt" in ov:
                arr = np.asarray(ov["rms_opt"], dtype=float)
                if arr.size > 0:
                    max_rmse = _clean_float(np.max(arr))

    elif isinstance(vlt3_data, dict):
        if "muscle_force_rmse_per_muscle" in vlt3_data and isinstance(
            vlt3_data["muscle_force_rmse_per_muscle"], dict
        ):
            for k, v in vlt3_data["muscle_force_rmse_per_muscle"].items():
                fv = _clean_float(v)
                if fv is not None:
                    per_muscle[str(k)] = fv
            if "muscle_force_rmse_max" in vlt3_data:
                max_rmse = _clean_float(vlt3_data["muscle_force_rmse_max"])
        elif "muscles" in vlt3_data and isinstance(vlt3_data["muscles"], dict):
            for k, v in vlt3_data["muscles"].items():
                fv = (
                    _extract_muscle_force_rmse(v)
                    if isinstance(v, dict)
                    else _clean_float(v)
                )
                if fv is not None:
                    per_muscle[str(k)] = fv
            ov_dict = vlt3_data.get("overall")
            if isinstance(ov_dict, dict) and "rms_opt" in ov_dict:
                arr = np.asarray(ov_dict["rms_opt"], dtype=float)
                if arr.size > 0:
                    max_rmse = _clean_float(np.max(arr))
        elif "rms_opt" in vlt3_data:
            arr = np.asarray(vlt3_data["rms_opt"], dtype=float)
            if arr.size > 0:
                max_rmse = _clean_float(np.max(arr))
        else:
            for k, v in vlt3_data.items():
                if isinstance(v, dict):
                    fv = _extract_muscle_force_rmse(v)
                elif isinstance(v, (int, float, np.floating, np.integer)):
                    fv = _clean_float(v)
                else:
                    fv = None
                if fv is not None:
                    per_muscle[str(k)] = fv

    if per_muscle:
        per_muscle_max = max(per_muscle.values())
        max_rmse = (
            per_muscle_max if max_rmse is None else max(max_rmse, per_muscle_max)
        )

    return {
        "muscle_force_rmse_per_muscle": per_muscle,
        "muscle_force_rmse_max": max_rmse,
    }


def _infer_conversion_steps(
    output_folder: Path, model_name: str, explicit_steps: list[int] | None
) -> list[int]:
    if explicit_steps is not None:
        return sorted({int(s) for s in explicit_steps})

    steps: set[int] = set()
    if (
        output_folder / f"{model_name}_cvt1.xml"
    ).is_file() or (output_folder / "Step1_xmlConvert").is_dir():
        steps.add(1)
    if (
        output_folder / f"{model_name}_cvt2.xml"
    ).is_file() or (output_folder / "Step2_muscleKinematics").is_dir():
        steps.add(2)
    if (
        output_folder / f"{model_name}_cvt3.xml"
    ).is_file() or (output_folder / "Step3_muscleKinetics").is_dir():
        steps.add(3)

    for cfg_rel in (
        "Step2_muscleKinematics/config.pkl",
        "Step3_muscleKinetics/config.pkl",
    ):
        cfg_file = output_folder / cfg_rel
        if cfg_file.is_file():
            try:
                with cfg_file.open("rb") as fp:
                    cfg_dict = pickle.load(fp)
                if isinstance(cfg_dict, dict) and "convert_steps" in cfg_dict:
                    steps.update(int(s) for s in cfg_dict["convert_steps"])
            except (OSError, pickle.UnpicklingError, EOFError, ValueError):
                pass

    return sorted(steps)


def export_model_card(
    output_folder: str | os.PathLike,
    model_name: str,
    osim_file: str | os.PathLike | None = None,
    mjcf_file: str | os.PathLike | None = None,
    vlt1_data: Any = None,
    vlt2_data: Any = None,
    vlt3_data: Any = None,
    *,
    convert_steps: list[int] | None = None,
    kinematic_rmse_tol: float = 0.01,
    moment_arm_rmse_tol: float = 0.01,
    muscle_force_rmse_tol: float = 0.25,
) -> Path:
    """Export ``<output_folder>/<model_name>_model_card.json`` summarizing conversion & validation.

    :param output_folder: Directory where ``<model_name>_model_card.json`` will be written
    :param model_name: Base identifier of the musculoskeletal model
    :param osim_file: Optional source OpenSim ``.osim`` file path
    :param mjcf_file: Optional converted MuJoCo ``.xml`` file path (auto-discovered if omitted)
    :param vlt1_data: Optional Step 1 validation dict or folder path (auto-discovered if omitted)
    :param vlt2_data: Optional Step 2 validation dict or folder path (auto-discovered if omitted)
    :param vlt3_data: Optional Step 3 validation dict or folder path (auto-discovered if omitted)
    :return: Path to the written ``<model_name>_model_card.json`` file
    """
    out_dir = Path(output_folder)
    out_dir.mkdir(parents=True, exist_ok=True)

    osim_path = Path(osim_file) if osim_file is not None else None
    mjcf_path = (
        Path(mjcf_file)
        if mjcf_file is not None
        else _discover_mjcf_file(out_dir, model_name)
    )

    steps_run = _infer_conversion_steps(out_dir, model_name, convert_steps)
    provenance = {
        "generator": "myoconverter",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "osim_file": str(osim_path) if osim_path is not None else None,
        "osim_sha256": _sha256_file(osim_path),
        "mjcf_file": str(mjcf_path) if mjcf_path is not None else None,
        "mjcf_sha256": _sha256_file(mjcf_path),
        "conversion_steps_run": steps_run,
    }

    model_summary = _extract_model_summary(mjcf_path)

    v1 = _load_vlt1(out_dir, vlt1_data)
    v2 = _load_vlt2(out_dir, vlt2_data)
    v3 = _load_vlt3(out_dir, vlt3_data)

    joint_rom_rad: dict[str, list[float]] = dict(
        model_summary.get("joint_ranges", {})
    )
    joint_rom_rad.update(v1["joint_rom_rad"])

    kinematic_rmse = v1["kinematic_rmse"]
    moment_arm_rmse_max = v2["moment_arm_rmse_max"]
    muscle_force_rmse_max = v3["muscle_force_rmse_max"]

    checks: list[bool] = []
    if kinematic_rmse is not None:
        checks.append(kinematic_rmse <= kinematic_rmse_tol)
    if moment_arm_rmse_max is not None:
        checks.append(moment_arm_rmse_max <= moment_arm_rmse_tol)
    if muscle_force_rmse_max is not None:
        checks.append(muscle_force_rmse_max <= muscle_force_rmse_tol)

    if checks:
        passed_quality_gate = all(checks)
    else:
        passed_quality_gate = bool(model_summary.get("compiled", False))

    validation = {
        "joint_rom_rad": joint_rom_rad,
        "kinematic_rmse": kinematic_rmse,
        "kinematic_rmse_std": v1["kinematic_rmse_std"],
        "moment_arm_rmse_per_muscle": v2["moment_arm_rmse_per_muscle"],
        "moment_arm_rmse_max": moment_arm_rmse_max,
        "muscle_force_rmse_per_muscle": v3["muscle_force_rmse_per_muscle"],
        "muscle_force_rmse_max": muscle_force_rmse_max,
        "quality_gate_thresholds": {
            "kinematic_rmse_tol": float(kinematic_rmse_tol),
            "moment_arm_rmse_tol": float(moment_arm_rmse_tol),
            "muscle_force_rmse_tol": float(muscle_force_rmse_tol),
        },
        "passed_quality_gate": passed_quality_gate,
    }

    card_payload = {
        "schema_version": SCHEMA_VERSION,
        "model_name": model_name,
        "provenance": provenance,
        "model_summary": model_summary,
        "validation": validation,
    }

    card_path = out_dir / f"{model_name}_model_card.json"
    card_path.write_text(json.dumps(card_payload, indent=2), encoding="utf-8")
    logger.info(f"Exported ModelCard v{SCHEMA_VERSION} to {card_path}")
    return card_path


def _build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export a MyoConverter JSON validation model card."
    )
    parser.add_argument(
        "output_folder",
        type=str,
        help="Conversion output folder containing Step1..3 artifacts",
    )
    parser.add_argument("model_name", type=str, help="Model base name")
    parser.add_argument(
        "--osim_file", type=str, default=None, help="Optional source .osim path"
    )
    parser.add_argument(
        "--mjcf_file", type=str, default=None, help="Optional converted .xml path"
    )
    return parser


def main(argv: list[str] | None = None) -> Path:
    args = _build_cli_parser().parse_args(argv)
    return export_model_card(
        output_folder=args.output_folder,
        model_name=args.model_name,
        osim_file=args.osim_file,
        mjcf_file=args.mjcf_file,
    )


if __name__ == "__main__":
    main()
