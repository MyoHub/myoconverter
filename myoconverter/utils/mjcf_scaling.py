"""Post-conversion OpenSim ``<ScaleSet>`` and anthropometric MJCF scaling utility.

Scales an already-converted MyoConverter MuJoCo ``.xml`` model from either an
OpenSim ``<ScaleSet>`` / ``<ScaleTool>`` XML file (parsed directly via
``lxml.etree`` with zero OpenSim C++ runtime dependency) or anthropometric and
biomechanical parameters (``height_scale``, ``mass_scale``, ``segment_scales``,
``strength_scale``, ``muscle_strength_scales``, ``passive_damping_scale``,
``passive_stiffness_scale``, ``joint_rom_scales``), producing a valid,
compilable subject-specific MJCF without re-running ``cvt2``/``cvt3``
optimization.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from loguru import logger
from lxml import etree
from scipy.spatial.transform import Rotation


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


def _str2arr(text: str) -> np.ndarray:
    return np.array([float(tok) for tok in text.split()], dtype=float)


def _arr2str(arr: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(v):.6g}" for v in arr)


def _normalize_scale3(val: Any) -> np.ndarray:
    if isinstance(val, (int, float, np.floating, np.integer)):
        f = float(val)
        vec = np.array([f, f, f], dtype=float)
    else:
        arr = np.asarray(val, dtype=float).reshape(-1)
        if arr.size == 1:
            vec = np.repeat(arr[0], 3)
        elif arr.size == 3:
            vec = arr.copy()
        else:
            raise ValueError(f"Expected scalar or 3-element scale vector, got {val!r}")
    if np.any(vec <= 0.0) or not np.all(np.isfinite(vec)):
        raise ValueError(
            f"Segment scale factors must be finite and positive, got {vec}"
        )
    return vec


def parse_opensim_scaleset(
    scale_set_xml: str | os.PathLike,
) -> dict[str, tuple[float, float, float]]:
    """Parse per-segment 3D scale factors from an OpenSim ``<ScaleSet>`` or ``<ScaleTool>`` XML file.

    Parses XML directly via ``lxml.etree`` without requiring OpenSim C++ bindings.

    :param scale_set_xml: Path to OpenSim ``ScaleSet`` / ``ScaleTool`` XML file
    :return: Mapping ``{segment_name: (sx, sy, sz)}`` for all enabled ``<Scale>`` entries
    """
    xml_path = Path(scale_set_xml)
    if not xml_path.is_file():
        raise FileNotFoundError(f"ScaleSet XML file not found: {xml_path}")

    parser = etree.XMLParser(remove_blank_text=True)
    root = etree.parse(str(xml_path), parser).getroot()

    scales: dict[str, tuple[float, float, float]] = {}
    for scale_el in root.xpath(".//Scale"):
        apply_el = scale_el.find("apply")
        if (
            apply_el is not None
            and apply_el.text is not None
            and apply_el.text.strip().lower() in {"false", "0", "no", "f"}
        ):
            continue

        seg_el = scale_el.find("segment")
        vals_el = scale_el.find("scales")
        if (
            seg_el is None
            or seg_el.text is None
            or vals_el is None
            or vals_el.text is None
        ):
            continue

        seg_name = seg_el.text.strip()
        if not seg_name:
            continue
        vec = _normalize_scale3(_str2arr(vals_el.text))
        scales[seg_name] = (float(vec[0]), float(vec[1]), float(vec[2]))

    return scales


def _element_rotation_matrix(el: etree._Element) -> np.ndarray:
    if "quat" in el.attrib:
        q = _str2arr(el.attrib["quat"])
        norm = np.linalg.norm(q)
        if norm > 0:
            q = q / norm
            # MuJoCo quaternion is scalar-first (w, x, y, z); scipy is (x, y, z, w)
            return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    if "euler" in el.attrib:
        eul = _str2arr(el.attrib["euler"])
        return Rotation.from_euler("XYZ", eul).as_matrix()
    return np.eye(3)


def _matrix_to_mjc_quat(rot_mat: np.ndarray) -> np.ndarray:
    if np.linalg.det(rot_mat) < 0:
        rot_mat = rot_mat.copy()
        rot_mat[:, 0] *= -1.0
    xyzw = Rotation.from_matrix(rot_mat).as_quat()
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=float)


def _scale_inertial_element(
    inertial_el: etree._Element, mass_ratio: float, seg_scale: np.ndarray
) -> None:
    """Scale ``<inertial>`` pos, mass, and inertia while preserving triangle inequalities."""
    if "pos" in inertial_el.attrib:
        inertial_el.attrib["pos"] = _arr2str(
            _str2arr(inertial_el.attrib["pos"]) * seg_scale
        )

    if "mass" in inertial_el.attrib:
        old_mass = float(inertial_el.attrib["mass"])
        new_mass = max(old_mass * mass_ratio, 1e-6)
        inertial_el.attrib["mass"] = f"{new_mass:.6g}"

    is_isotropic = bool(np.allclose(seg_scale, seg_scale[0]))
    s_mat = np.diag(seg_scale)

    if "diaginertia" in inertial_el.attrib:
        diag = _str2arr(inertial_el.attrib["diaginertia"])
        if is_isotropic:
            new_diag = np.maximum(diag * mass_ratio * (seg_scale[0] ** 2), 1e-9)
            inertial_el.attrib["diaginertia"] = _arr2str(new_diag)
        else:
            r_in = _element_rotation_matrix(inertial_el)
            i_body = r_in @ np.diag(diag) @ r_in.T
            j_cov = 0.5 * np.trace(i_body) * np.eye(3) - i_body
            eigvals, eigvecs = np.linalg.eigh(j_cov)
            j_cov = eigvecs @ np.diag(np.maximum(eigvals, 1e-9)) @ eigvecs.T
            j_scaled = mass_ratio * (s_mat @ j_cov @ s_mat)
            i_scaled = np.trace(j_scaled) * np.eye(3) - j_scaled

            if "quat" not in inertial_el.attrib and "euler" not in inertial_el.attrib:
                new_diag = np.maximum(np.diag(i_scaled), 1e-9)
                inertial_el.attrib["diaginertia"] = _arr2str(new_diag)
            else:
                p_vals, p_vecs = np.linalg.eigh(i_scaled)
                order = np.argsort(p_vals)[::-1]
                p_vals = np.maximum(p_vals[order], 1e-9)
                p_vecs = p_vecs[:, order]
                inertial_el.attrib["diaginertia"] = _arr2str(p_vals)
                inertial_el.attrib["quat"] = _arr2str(_matrix_to_mjc_quat(p_vecs))
                inertial_el.attrib.pop("euler", None)

    elif "fullinertia" in inertial_el.attrib:
        vec = _str2arr(inertial_el.attrib["fullinertia"])
        i_body = np.array(
            [
                [vec[0], vec[3], vec[4]],
                [vec[3], vec[1], vec[5]],
                [vec[4], vec[5], vec[2]],
            ],
            dtype=float,
        )
        j_cov = 0.5 * np.trace(i_body) * np.eye(3) - i_body
        eigvals, eigvecs = np.linalg.eigh(j_cov)
        j_cov = eigvecs @ np.diag(np.maximum(eigvals, 1e-9)) @ eigvecs.T
        j_scaled = mass_ratio * (s_mat @ j_cov @ s_mat)
        i_scaled = np.trace(j_scaled) * np.eye(3) - j_scaled
        new_full = np.array(
            [
                i_scaled[0, 0],
                i_scaled[1, 1],
                i_scaled[2, 2],
                i_scaled[0, 1],
                i_scaled[0, 2],
                i_scaled[1, 2],
            ],
            dtype=float,
        )
        inertial_el.attrib["fullinertia"] = _arr2str(new_full)


def _scale_geom_element(
    geom_el: etree._Element,
    seg_scale: np.ndarray,
    asset_el: etree._Element | None,
    scaled_meshes: dict[str, np.ndarray],
) -> None:
    gtype = (
        geom_el.attrib.get("type", "sphere" if "mesh" not in geom_el.attrib else "mesh")
        .strip()
        .lower()
    )
    if gtype == "plane":
        return

    if "pos" in geom_el.attrib:
        geom_el.attrib["pos"] = _arr2str(_str2arr(geom_el.attrib["pos"]) * seg_scale)

    if "fromto" in geom_el.attrib:
        ft = _str2arr(geom_el.attrib["fromto"])
        if ft.size == 6:
            ft[:3] *= seg_scale
            ft[3:] *= seg_scale
            geom_el.attrib["fromto"] = _arr2str(ft)

    if "mesh" in geom_el.attrib and asset_el is not None:
        mesh_name = geom_el.attrib["mesh"]
        mesh_el = asset_el.find(f"mesh[@name='{mesh_name}']")
        if mesh_el is not None:
            if mesh_name not in scaled_meshes:
                old_mesh_scale = _str2arr(mesh_el.attrib.get("scale", "1 1 1"))
                mesh_el.attrib["scale"] = _arr2str(old_mesh_scale * seg_scale)
                scaled_meshes[mesh_name] = seg_scale.copy()
            elif not np.allclose(scaled_meshes[mesh_name], seg_scale):
                body_name = geom_el.getparent().attrib.get("name", "scaled")
                clone_name = f"{mesh_name}_{body_name}"
                if asset_el.find(f"mesh[@name='{clone_name}']") is None:
                    clone_el = etree.SubElement(
                        asset_el, "mesh", attrib=dict(mesh_el.attrib)
                    )
                    clone_el.attrib["name"] = clone_name
                    orig_scale = (
                        _str2arr(mesh_el.attrib["scale"]) / scaled_meshes[mesh_name]
                    )
                    clone_el.attrib["scale"] = _arr2str(orig_scale * seg_scale)
                    scaled_meshes[clone_name] = seg_scale.copy()
                geom_el.attrib["mesh"] = clone_name
        return

    if "size" not in geom_el.attrib:
        return

    sz = _str2arr(geom_el.attrib["size"])
    r_geom = _element_rotation_matrix(geom_el)
    local_scales = np.linalg.norm(np.diag(seg_scale) @ r_geom, axis=0)
    iso_scale = float(np.cbrt(np.prod(seg_scale)))

    if gtype == "sphere":
        sz[0] *= iso_scale
    elif gtype in {"cylinder", "capsule"}:
        radial_scale = float(np.sqrt(local_scales[0] * local_scales[1]))
        sz[0] *= radial_scale
        if sz.size >= 2:
            sz[1] *= float(local_scales[2])
    elif gtype in {"ellipsoid", "box"}:
        n = min(sz.size, 3)
        sz[:n] *= local_scales[:n]
    else:
        sz *= iso_scale

    geom_el.attrib["size"] = _arr2str(sz)


def _measure_tendon_lengths(root: etree._Element) -> dict[str, float]:
    xml_str = etree.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml_str)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lengths: dict[str, float] = {}
    for tid in range(model.ntendon):
        tname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tid)
        if tname:
            lengths[tname] = float(data.ten_length[tid])
    return lengths


def _unpack_scale_dict(scale_params: dict[str, Any] | None) -> dict[str, Any]:
    if not scale_params:
        return {}
    out: dict[str, Any] = {}
    for k in (
        "height_scale",
        "mass_scale",
        "segment_scales",
        "strength_scale",
        "muscle_strength_scales",
        "passive_damping_scale",
        "passive_stiffness_scale",
        "joint_rom_scales",
    ):
        if k in scale_params and scale_params[k] is not None:
            out[k] = scale_params[k]
    return out


def scale_converted_mjcf(
    mjcf_file: str | os.PathLike,
    scale_spec: str | os.PathLike | dict[str, Any] | None = None,
    output_xml: str | os.PathLike | None = None,
    *,
    scale_set_xml: str | os.PathLike | None = None,
    scale_params: dict[str, Any] | None = None,
    height_scale: float = 1.0,
    mass_scale: float | None = None,
    segment_scales: dict[str, Any] | None = None,
    strength_scale: float = 1.0,
    muscle_strength_scales: dict[str, float] | None = None,
    passive_damping_scale: float = 1.0,
    passive_stiffness_scale: float = 1.0,
    joint_rom_scales: dict[str, Any] | float | None = None,
) -> Path:
    """Scale an already-converted MuJoCo ``.xml`` model anthropometrically and biomechanically.

    :param mjcf_file: Input converted MuJoCo ``.xml`` path
    :param scale_spec: Optional parameter dict, OpenSim ``<ScaleSet>`` XML path,
        or destination ``output_xml`` path when passed positionally
    :param output_xml: Destination ``.xml`` path (defaults to ``<stem>_scaled.xml`` alongside ``mjcf_file``)
    :param scale_set_xml: Optional OpenSim ``<ScaleSet>`` XML file path
    :param scale_params: Optional anthropometric/biomechanical parameter dictionary
    :param height_scale: Global uniform stature scale factor
    :param mass_scale: Optional total body mass scale factor (defaults to volumetric scaling if ``None``)
    :param segment_scales: Optional ``{body_name: (sx, sy, sz) | float}`` per-body scale overrides
    :param strength_scale: Global muscle peak isometric force multiplier
    :param muscle_strength_scales: Optional ``{muscle_name: float}`` per-muscle strength multipliers
    :param passive_damping_scale: Multiplier for passive joint damping
    :param passive_stiffness_scale: Multiplier for passive joint stiffness / soft-limit stiffness
    :param joint_rom_scales: Scalar or ``{joint_name: float | (low, high)}`` joint ROM scale(s)
    :return: Path to the written and MuJoCo-validated scaled ``.xml`` file
    """
    in_path = Path(mjcf_file)
    if not in_path.is_file():
        raise FileNotFoundError(f"Input MJCF file not found: {in_path}")

    if isinstance(scale_spec, dict):
        if scale_params is None:
            scale_params = scale_spec
    elif scale_spec is not None:
        if output_xml is not None:
            if scale_set_xml is None:
                scale_set_xml = scale_spec
        else:
            spec_path = Path(scale_spec)
            if scale_set_xml is None and spec_path.is_file():
                head = spec_path.read_text(encoding="utf-8", errors="ignore")[:4096]
                if (
                    "<ScaleSet" in head
                    or "<ScaleTool" in head
                    or "<Scale " in head
                    or "<Scale>" in head
                ):
                    scale_set_xml = scale_spec
                else:
                    output_xml = scale_spec
            else:
                output_xml = scale_spec

    cfg = _unpack_scale_dict(scale_params)
    eff_height = float(
        cfg.get("height_scale", height_scale) if height_scale == 1.0 else height_scale
    )
    eff_mass_scale = (
        mass_scale if mass_scale is not None else cfg.get("mass_scale", None)
    )
    eff_strength = float(
        cfg.get("strength_scale", strength_scale)
        if strength_scale == 1.0
        else strength_scale
    )
    eff_damping_scale = float(
        cfg.get("passive_damping_scale", passive_damping_scale)
        if passive_damping_scale == 1.0
        else passive_damping_scale
    )
    eff_stiffness_scale = float(
        cfg.get("passive_stiffness_scale", passive_stiffness_scale)
        if passive_stiffness_scale == 1.0
        else passive_stiffness_scale
    )
    eff_rom_scales = (
        joint_rom_scales
        if joint_rom_scales is not None
        else cfg.get("joint_rom_scales", None)
    )

    eff_muscle_scales: dict[str, float] = {}
    if isinstance(cfg.get("muscle_strength_scales"), dict):
        eff_muscle_scales.update(
            {str(k): float(v) for k, v in cfg["muscle_strength_scales"].items()}
        )
    if muscle_strength_scales:
        eff_muscle_scales.update(
            {str(k): float(v) for k, v in muscle_strength_scales.items()}
        )

    combined_seg_scales: dict[str, np.ndarray] = {}
    if scale_set_xml is not None:
        for seg_name, s3 in parse_opensim_scaleset(scale_set_xml).items():
            combined_seg_scales[seg_name] = _normalize_scale3(s3)
    if isinstance(cfg.get("segment_scales"), dict):
        for seg_name, s3 in cfg["segment_scales"].items():
            vec = _normalize_scale3(s3)
            combined_seg_scales[seg_name] = (
                combined_seg_scales.get(seg_name, np.ones(3)) * vec
            )
    if segment_scales:
        for seg_name, s3 in segment_scales.items():
            vec = _normalize_scale3(s3)
            combined_seg_scales[seg_name] = (
                combined_seg_scales.get(seg_name, np.ones(3)) * vec
            )

    if eff_height <= 0.0 or not math.isfinite(eff_height):
        raise ValueError(f"height_scale must be finite and positive, got {eff_height}")
    if eff_mass_scale is not None and (
        float(eff_mass_scale) <= 0.0 or not math.isfinite(float(eff_mass_scale))
    ):
        raise ValueError(
            f"mass_scale must be finite and positive, got {eff_mass_scale}"
        )
    if eff_strength <= 0.0 or not math.isfinite(eff_strength):
        raise ValueError(
            f"strength_scale must be finite and positive, got {eff_strength}"
        )

    out_path = (
        Path(output_xml)
        if output_xml is not None
        else in_path.with_name(f"{in_path.stem}_scaled.xml")
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(str(in_path), parser)
    root = _sanitize_mjcf_tree_for_mujoco3(tree, base_dir=in_path.parent)

    orig_tendon_lengths = _measure_tendon_lengths(root)

    worldbody = root.find("worldbody")
    asset_el = root.find("asset")
    default_joint_el = root.find("default/joint")
    default_damping = (
        float(default_joint_el.attrib.get("damping", "0"))
        if default_joint_el is not None
        else 0.0
    )
    default_stiffness = (
        float(default_joint_el.attrib.get("stiffness", "0"))
        if default_joint_el is not None
        else 0.0
    )
    default_limited = (
        default_joint_el.attrib.get("limited", "false").lower() == "true"
        if default_joint_el is not None
        else False
    )

    base_scale_vec = np.array([eff_height, eff_height, eff_height], dtype=float)
    body_scales: dict[str, np.ndarray] = {}
    if worldbody is not None:
        for body_el in worldbody.findall(".//body"):
            bname = body_el.attrib.get("name", "")
            body_scales[bname] = base_scale_vec * combined_seg_scales.get(
                bname, np.ones(3)
            )

    orig_masses: list[tuple[etree._Element, float, np.ndarray]] = []
    if worldbody is not None:
        for body_el in worldbody.findall(".//body"):
            bname = body_el.attrib.get("name", "")
            if bname == "ground":
                continue
            in_el = body_el.find("inertial")
            if in_el is not None and "mass" in in_el.attrib:
                m_val = float(in_el.attrib["mass"])
                s_vec = body_scales.get(bname, base_scale_vec)
                orig_masses.append((in_el, m_val, s_vec))

    total_orig_mass = sum(m for _, m, _ in orig_masses)
    total_vol_mass = sum(m * float(np.prod(s)) for _, m, s in orig_masses)
    if eff_mass_scale is not None and total_orig_mass > 0.0 and total_vol_mass > 0.0:
        target_total_mass = float(eff_mass_scale) * total_orig_mass
        density_factor = target_total_mass / total_vol_mass
    elif eff_mass_scale is not None:
        density_factor = float(eff_mass_scale)
    else:
        density_factor = 1.0

    for in_el, m_val, s_vec in orig_masses:
        vol_ratio = float(np.prod(s_vec))
        mass_ratio = (
            density_factor * vol_ratio if m_val > 0.0 else float(eff_mass_scale or 1.0)
        )
        _scale_inertial_element(in_el, mass_ratio, s_vec)

    scaled_meshes: dict[str, np.ndarray] = {}
    site_scale_lookup: dict[str, float] = {}
    tightened_joint_ranges: dict[str, tuple[float, float]] = {}

    def _walk_body(body_el: etree._Element, parent_scale: np.ndarray) -> None:
        bname = body_el.attrib.get("name", "")
        cur_scale = body_scales.get(bname, base_scale_vec)

        if bname != "ground" and "pos" in body_el.attrib:
            body_el.attrib["pos"] = _arr2str(
                _str2arr(body_el.attrib["pos"]) * parent_scale
            )

        for geom_el in body_el.findall("geom"):
            _scale_geom_element(geom_el, cur_scale, asset_el, scaled_meshes)

        for site_el in body_el.findall("site"):
            if "pos" in site_el.attrib:
                site_el.attrib["pos"] = _arr2str(
                    _str2arr(site_el.attrib["pos"]) * cur_scale
                )
            sname = site_el.attrib.get("name")
            if sname:
                site_scale_lookup[sname] = float(np.cbrt(np.prod(cur_scale)))

        for jnt_el in body_el.findall("joint"):
            jtype = jnt_el.attrib.get("type", "hinge").strip().lower()
            if jtype == "free":
                continue
            jname = jnt_el.attrib.get("name", "")

            if "pos" in jnt_el.attrib:
                jnt_el.attrib["pos"] = _arr2str(
                    _str2arr(jnt_el.attrib["pos"]) * cur_scale
                )

            if eff_damping_scale != 1.0 or "damping" in jnt_el.attrib:
                cur_damp = float(jnt_el.attrib.get("damping", default_damping))
                jnt_el.attrib["damping"] = f"{cur_damp * eff_damping_scale:.6g}"

            if eff_stiffness_scale != 1.0 or "stiffness" in jnt_el.attrib:
                cur_stiff = float(jnt_el.attrib.get("stiffness", default_stiffness))
                if cur_stiff != 0.0 or "stiffness" in jnt_el.attrib:
                    jnt_el.attrib["stiffness"] = (
                        f"{cur_stiff * eff_stiffness_scale:.6g}"
                    )

            if "solreflimit" in jnt_el.attrib and (
                eff_stiffness_scale != 1.0 or eff_damping_scale != 1.0
            ):
                sref = _str2arr(jnt_el.attrib["solreflimit"])
                if sref.size == 2 and sref[0] < 0.0 and sref[1] <= 0.0:
                    sref[0] *= eff_stiffness_scale
                    sref[1] *= eff_damping_scale
                    jnt_el.attrib["solreflimit"] = _arr2str(sref)

            is_lim = (
                jnt_el.attrib.get("limited", "true" if default_limited else "false")
                .lower()
                == "true"
            )
            if "range" in jnt_el.attrib and is_lim:
                low, high = [float(v) for v in _str2arr(jnt_el.attrib["range"])]
                if jtype == "slide":
                    axis = _str2arr(jnt_el.attrib.get("axis", "1 0 0"))
                    axis_norm = np.linalg.norm(axis)
                    axis_unit = (
                        axis / axis_norm
                        if axis_norm > 0
                        else np.array([1.0, 0.0, 0.0])
                    )
                    slide_scale = float(np.linalg.norm(cur_scale * axis_unit))
                    low *= slide_scale
                    high *= slide_scale

                rom_entry = None
                if isinstance(eff_rom_scales, (int, float, np.floating, np.integer)):
                    rom_entry = float(eff_rom_scales)
                elif isinstance(eff_rom_scales, dict) and jname in eff_rom_scales:
                    rom_entry = eff_rom_scales[jname]

                if rom_entry is not None:
                    if (
                        isinstance(rom_entry, (list, tuple, np.ndarray))
                        and len(rom_entry) == 2
                    ):
                        low, high = float(rom_entry[0]), float(rom_entry[1])
                    else:
                        rscale = float(rom_entry)
                        if rscale <= 0.0:
                            raise ValueError(
                                f"ROM scale for joint '{jname}' must be positive, got {rscale}"
                            )
                        ref_val = float(jnt_el.attrib.get("ref", "0"))
                        anchor = float(np.clip(ref_val, low, high))
                        low = anchor + rscale * (low - anchor)
                        high = anchor + rscale * (high - anchor)
                    if low >= high:
                        mid = 0.5 * (low + high)
                        low, high = mid - 1e-5, mid + 1e-5

                jnt_el.attrib["range"] = f"{low:.6g} {high:.6g}"
                tightened_joint_ranges[jname] = (low, high)

        for child_body in body_el.findall("body"):
            _walk_body(child_body, cur_scale)

    if worldbody is not None:
        for top_body in worldbody.findall("body"):
            _walk_body(top_body, base_scale_vec)

    new_tendon_lengths = _measure_tendon_lengths(root)
    tendon_ratios: dict[str, float] = {}
    tendon_section = root.find("tendon")
    if tendon_section is not None:
        for sp_el in tendon_section.findall("spatial"):
            tname = sp_el.attrib.get("name", "")
            old_l = orig_tendon_lengths.get(tname, 0.0)
            new_l = new_tendon_lengths.get(tname, 0.0)
            if old_l > 1e-6 and new_l > 1e-6:
                ratio = new_l / old_l
            else:
                site_scales = [
                    site_scale_lookup[s.attrib["site"]]
                    for s in sp_el.findall("site")
                    if s.attrib.get("site") in site_scale_lookup
                ]
                ratio = float(np.mean(site_scales)) if site_scales else eff_height
            tendon_ratios[tname] = ratio

            if "springlength" in sp_el.attrib:
                sl = _str2arr(sp_el.attrib["springlength"])
                sp_el.attrib["springlength"] = _arr2str(sl * ratio)

    actuator_section = root.find("actuator")
    if actuator_section is not None:
        for act_el in actuator_section:
            if not isinstance(act_el.tag, str):
                continue
            aname = act_el.attrib.get("name", "")
            is_muscle = (
                act_el.tag == "muscle"
                or act_el.attrib.get("class") == "muscle"
                or act_el.attrib.get("dyntype") == "muscle"
                or act_el.attrib.get("gaintype") == "muscle"
            )
            if not is_muscle:
                continue

            tname = act_el.attrib.get("tendon", "")
            l_ratio = tendon_ratios.get(tname, eff_height)
            if "lengthrange" in act_el.attrib:
                lr = _str2arr(act_el.attrib["lengthrange"])
                act_el.attrib["lengthrange"] = _arr2str(lr * l_ratio)

            f_mult = eff_strength * float(eff_muscle_scales.get(aname, 1.0))
            if f_mult != 1.0:
                updated_prm = False
                for prm_attr in ("gainprm", "biasprm"):
                    if prm_attr in act_el.attrib:
                        prm = _str2arr(act_el.attrib[prm_attr])
                        if prm.size >= 4:
                            if prm[2] >= 0.0:
                                prm[2] *= f_mult
                            else:
                                prm[3] *= f_mult
                            act_el.attrib[prm_attr] = _arr2str(prm)
                            updated_prm = True
                if not updated_prm:
                    if (
                        "force" in act_el.attrib
                        and float(act_el.attrib["force"]) >= 0.0
                    ):
                        act_el.attrib["force"] = (
                            f"{float(act_el.attrib['force']) * f_mult:.6g}"
                        )
                    elif "scale" in act_el.attrib:
                        act_el.attrib["scale"] = (
                            f"{float(act_el.attrib['scale']) * f_mult:.6g}"
                        )
                    else:
                        default_muscle = root.find(".//default[@class='muscle']/muscle")
                        base_scale = (
                            float(default_muscle.attrib.get("scale", "200"))
                            if default_muscle is not None
                            else 200.0
                        )
                        act_el.attrib["scale"] = f"{base_scale * f_mult:.6g}"

    compiler_el = root.find("compiler")
    same_dir = out_path.parent.resolve() == in_path.parent.resolve()
    has_removable_meshdir = (
        compiler_el is not None
        and same_dir
        and compiler_el.attrib.get("meshdir") == str(in_path.parent.resolve())
        and (
            (out_path.parent / "Geometry").is_dir()
            or asset_el is None
            or asset_el.find("mesh") is None
        )
    )
    if has_removable_meshdir:
        compiler_el.attrib.pop("meshdir", None)

    keyframe_el = root.find("keyframe")
    if keyframe_el is not None and tightened_joint_ranges:
        tmp_model = mujoco.MjModel.from_xml_string(
            etree.tostring(root, encoding="unicode")
        )
        for key_el in keyframe_el.findall("key"):
            if "qpos" in key_el.attrib:
                qpos = _str2arr(key_el.attrib["qpos"])
                if qpos.size == tmp_model.nq:
                    for jname, (lo, hi) in tightened_joint_ranges.items():
                        jid = mujoco.mj_name2id(
                            tmp_model, mujoco.mjtObj.mjOBJ_JOINT, jname
                        )
                        if jid >= 0 and int(tmp_model.jnt_type[jid]) in {
                            int(mujoco.mjtJoint.mjJNT_HINGE),
                            int(mujoco.mjtJoint.mjJNT_SLIDE),
                        }:
                            qadr = int(tmp_model.jnt_qposadr[jid])
                            qpos[qadr] = float(np.clip(qpos[qadr], lo, hi))
                    key_el.attrib["qpos"] = _arr2str(qpos)

    tree.write(str(out_path), pretty_print=True, xml_declaration=True, encoding="utf-8")

    compiled = mujoco.MjModel.from_xml_path(str(out_path))
    data = mujoco.MjData(compiled)
    mujoco.mj_forward(compiled, data)
    logger.info(
        f"Scaled MJCF written to {out_path} (total_mass={float(np.sum(compiled.body_mass[1:])):.3f} kg, "
        f"nq={compiled.nq}, nu={compiled.nu})"
    )
    return out_path


def _build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Scale a converted MyoConverter MuJoCo XML model using OpenSim ScaleSet or anthropometric parameters."
    )
    parser.add_argument("mjcf_file", type=str, help="Path to converted MuJoCo .xml file")
    parser.add_argument("--output_xml", type=str, default=None, help="Output scaled .xml path")
    parser.add_argument(
        "--scale_set_xml",
        type=str,
        default=None,
        help="Optional OpenSim <ScaleSet> XML path",
    )
    parser.add_argument(
        "--scale_json",
        type=str,
        default=None,
        help="Optional JSON file with anthropometric and biomechanical scaling parameters",
    )
    parser.add_argument(
        "--height_scale", type=float, default=1.0, help="Global stature scale factor"
    )
    parser.add_argument(
        "--mass_scale", type=float, default=None, help="Total body mass scale factor"
    )
    parser.add_argument(
        "--strength_scale",
        type=float,
        default=1.0,
        help="Global muscle strength multiplier",
    )
    parser.add_argument(
        "--passive_damping_scale",
        type=float,
        default=1.0,
        help="Passive joint damping multiplier",
    )
    parser.add_argument(
        "--passive_stiffness_scale",
        type=float,
        default=1.0,
        help="Passive joint stiffness multiplier",
    )
    parser.add_argument(
        "--joint_rom_scale",
        type=float,
        default=None,
        help="Uniform joint ROM scale factor",
    )
    return parser


def main(argv: list[str] | None = None) -> Path:
    args = _build_cli_parser().parse_args(argv)
    scale_params = None
    if args.scale_json is not None:
        scale_params = json.loads(Path(args.scale_json).read_text(encoding="utf-8"))
    return scale_converted_mjcf(
        mjcf_file=args.mjcf_file,
        output_xml=args.output_xml,
        scale_set_xml=args.scale_set_xml,
        scale_params=scale_params,
        height_scale=args.height_scale,
        mass_scale=args.mass_scale,
        strength_scale=args.strength_scale,
        passive_damping_scale=args.passive_damping_scale,
        passive_stiffness_scale=args.passive_stiffness_scale,
        joint_rom_scales=args.joint_rom_scale,
    )


if __name__ == "__main__":
    main()
