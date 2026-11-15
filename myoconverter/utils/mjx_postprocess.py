"""MJX and GPU-RL post-processor for converted MyoConverter MuJoCo XML models.

Prepares converted musculoskeletal MJCFs for stable reinforcement learning and
GPU-batched MJX simulation by:
1. Enforcing minimum passive ``damping`` and ``armature`` floors on non-free joints,
2. Replacing MJX-unsupported wrap geometries (``ellipsoid``, ``torus``, etc.) with
   best-fit ``sphere`` or oriented ``cylinder`` primitives and recording every
   geometric approximation in a structured report dictionary,
3. Configuring MJX-compatible ``<option>`` solver settings (``Euler`` or
   ``implicitfast``, ``iterations``, ``ls_iterations``), and
4. Compiling the resulting XML via ``mujoco.MjModel.from_xml_path(...)`` to
   verify schema and numerical validity.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from loguru import logger
from lxml import etree
from scipy.spatial.transform import Rotation

_VALID_MJX_INTEGRATORS = {"Euler", "implicitfast", "implicit", "RK4"}


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


def _geom_rotation_matrix(geom_el: etree._Element) -> np.ndarray:
    if "quat" in geom_el.attrib:
        q = _str2arr(geom_el.attrib["quat"])
        norm = np.linalg.norm(q)
        if norm > 0:
            q = q / norm
            return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    if "euler" in geom_el.attrib:
        return Rotation.from_euler("XYZ", _str2arr(geom_el.attrib["euler"])).as_matrix()
    return np.eye(3)


def _set_geom_rotation_matrix(geom_el: etree._Element, rot_mat: np.ndarray) -> None:
    if np.linalg.det(rot_mat) < 0:
        rot_mat = rot_mat.copy()
        rot_mat[:, 0] *= -1.0
    xyzw = Rotation.from_matrix(rot_mat).as_quat()
    wxyz = np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=float)
    geom_el.attrib["quat"] = _arr2str(wxyz)
    geom_el.attrib.pop("euler", None)
    geom_el.attrib.pop("axisangle", None)
    geom_el.attrib.pop("xyaxes", None)
    geom_el.attrib.pop("zaxis", None)


def _fit_ellipsoid_to_primitive(
    radii: np.ndarray,
    aspect_threshold: float = 1.35,
) -> tuple[str, np.ndarray, np.ndarray]:
    """Fit 3D ellipsoid semi-axes ``(rx, ry, rz)`` to a ``sphere`` or locally rotated ``cylinder``.

    :return: ``(primitive_type, size_array, local_rotation_delta_3x3)``
    """
    r = np.maximum(np.asarray(radii, dtype=float).reshape(-1), 1e-6)
    if r.size == 1:
        return "sphere", np.array([float(r[0])]), np.eye(3)
    if r.size == 2:
        return "cylinder", np.array([float(r[0]), float(r[1])]), np.eye(3)

    rx, ry, rz = float(r[0]), float(r[1]), float(r[2])
    aspect = max(rx, ry, rz) / max(min(rx, ry, rz), 1e-9)
    if aspect < aspect_threshold:
        r_eq = float(np.cbrt(rx * ry * rz))
        return "sphere", np.array([r_eq]), np.eye(3)

    # Identify principal symmetry / outlier axis whose log-radius is furthest from the other two
    log_r = np.log([rx, ry, rz])
    pair_diffs = [
        abs(log_r[1] - log_r[2]),  # axis 0 as cylinder axis (transverse: y, z)
        abs(log_r[0] - log_r[2]),  # axis 1 as cylinder axis (transverse: x, z)
        abs(log_r[0] - log_r[1]),  # axis 2 as cylinder axis (transverse: x, y)
    ]
    axis_idx = int(np.argmin(pair_diffs))
    other_axes = [i for i in range(3) if i != axis_idx]
    r_perp = float(np.sqrt(r[other_axes[0]] * r[other_axes[1]]))
    half_len = float(r[axis_idx])

    if axis_idx == 2:
        delta_rot = np.eye(3)
    elif axis_idx == 0:
        # Map cylinder local +Z onto original local +X
        delta_rot = Rotation.from_euler("y", 90.0, degrees=True).as_matrix()
    else:
        # Map cylinder local +Z onto original local +Y
        delta_rot = Rotation.from_euler("x", -90.0, degrees=True).as_matrix()

    return "cylinder", np.array([r_perp, half_len]), delta_rot


def _adjust_sidesites_outside_primitive(
    geom_el: etree._Element,
    sidesite_names: list[str],
    prim_type: str,
    prim_size: np.ndarray,
) -> None:
    """Ensure wrap sidesites that were outside an ellipsoid remain outside the fitted sphere/cylinder."""
    body_el = geom_el.getparent()
    if body_el is None or not sidesite_names:
        return

    g_pos = _str2arr(geom_el.attrib.get("pos", "0 0 0"))
    g_rot = _geom_rotation_matrix(geom_el)
    radius = float(prim_size[0])
    min_dist = radius + 0.001

    for sname in sidesite_names:
        site_el = body_el.find(f"site[@name='{sname}']")
        if site_el is None or "pos" not in site_el.attrib:
            continue
        s_pos = _str2arr(site_el.attrib["pos"])
        rel_local = g_rot.T @ (s_pos - g_pos)

        if prim_type == "sphere":
            dist = float(np.linalg.norm(rel_local))
            if 1e-7 < dist < min_dist:
                rel_local = rel_local * (min_dist / dist)
                site_el.attrib["pos"] = _arr2str(g_pos + g_rot @ rel_local)
        elif prim_type == "cylinder":
            radial = float(np.linalg.norm(rel_local[:2]))
            if 1e-7 < radial < min_dist:
                rel_local[:2] = rel_local[:2] * (min_dist / radial)
                site_el.attrib["pos"] = _arr2str(g_pos + g_rot @ rel_local)


def make_mjx_ready(
    mjcf_file: str | os.PathLike,
    output_xml: str | os.PathLike | None = None,
    *,
    min_damping: float = 0.05,
    min_armature: float = 0.005,
    configure_solver: bool = True,
    integrator: str = "implicitfast",
    iterations: int | None = 20,
    ls_iterations: int | None = 20,
    timestep: float | None = None,
    ellipsoid_aspect_threshold: float = 1.35,
) -> dict[str, Any]:
    """Post-process a converted MuJoCo ``.xml`` for stable RL and MJX batch simulation.

    :param mjcf_file: Path to input MuJoCo ``.xml`` file
    :param output_xml: Optional output ``.xml`` path (defaults to ``<stem>_mjx.xml``)
    :param min_damping: Minimum passive ``damping`` floor enforced on all non-free joints
    :param min_armature: Minimum rotor/joint ``armature`` floor enforced on all non-free joints
    :param configure_solver: Whether to configure MJX-compatible ``<option>`` solver attributes
    :param integrator: MJX integrator (``"Euler"`` or ``"implicitfast"``)
    :param iterations: Optional solver ``iterations`` count
    :param ls_iterations: Optional line-search ``ls_iterations`` count
    :param timestep: Optional simulation ``timestep`` override
    :param ellipsoid_aspect_threshold: Semi-axis aspect ratio above which ellipsoids map to ``cylinder`` vs ``sphere``
    :return: Structured report dictionary detailing all wrap approximations, joint floors, and model stats
    """
    in_path = Path(mjcf_file)
    if not in_path.is_file():
        raise FileNotFoundError(f"Input MJCF file not found: {in_path}")

    if min_damping < 0.0 or min_armature < 0.0:
        raise ValueError("min_damping and min_armature must be non-negative")
    if configure_solver and integrator not in _VALID_MJX_INTEGRATORS:
        raise ValueError(
            f"Unsupported integrator {integrator!r}; expected one of {sorted(_VALID_MJX_INTEGRATORS)}"
        )

    out_path = (
        Path(output_xml)
        if output_xml is not None
        else in_path.with_name(f"{in_path.stem}_mjx.xml")
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(str(in_path), parser)
    root = _sanitize_mjcf_tree_for_mujoco3(tree, base_dir=in_path.parent)

    # Consolidate <option> elements into a single clean <option> element
    option_elems = root.findall("option")
    if option_elems:
        primary_opt = option_elems[0]
        for extra_opt in option_elems[1:]:
            for k, v in extra_opt.attrib.items():
                primary_opt.attrib[k] = v
            root.remove(extra_opt)
    else:
        primary_opt = etree.Element("option")
        compiler_el = root.find("compiler")
        insert_idx = (
            (list(root).index(compiler_el) + 1) if compiler_el is not None else 0
        )
        root.insert(insert_idx, primary_opt)

    solver_options: dict[str, Any] = {}
    if configure_solver:
        primary_opt.attrib["integrator"] = integrator
        solver_options["integrator"] = integrator
        if iterations is not None:
            primary_opt.attrib["iterations"] = str(int(iterations))
            solver_options["iterations"] = int(iterations)
        if ls_iterations is not None:
            primary_opt.attrib["ls_iterations"] = str(int(ls_iterations))
            solver_options["ls_iterations"] = int(ls_iterations)
        if timestep is not None:
            primary_opt.attrib["timestep"] = f"{float(timestep):.6g}"
            solver_options["timestep"] = float(timestep)
        elif "timestep" in primary_opt.attrib:
            solver_options["timestep"] = float(primary_opt.attrib["timestep"])

    # Read <default><joint> baseline and enforce min_damping / min_armature strictly on non-free joints
    default_joint = root.find("default/joint")
    base_default_damping = 0.0
    base_default_armature = 0.0
    if default_joint is not None:
        base_default_damping = float(default_joint.attrib.get("damping", "0"))
        base_default_armature = float(default_joint.attrib.get("armature", "0"))

    joints_updated: list[dict[str, Any]] = []
    worldbody = root.find("worldbody")
    if worldbody is not None:
        for jnt_el in worldbody.findall(".//joint"):
            jtype = jnt_el.attrib.get("type", "hinge").strip().lower()
            if jtype == "free":
                if "damping" not in jnt_el.attrib:
                    jnt_el.attrib["damping"] = "0"
                if "armature" not in jnt_el.attrib:
                    jnt_el.attrib["armature"] = "0"
                continue

            jname = jnt_el.attrib.get("name", "<unnamed>")
            old_damp = float(jnt_el.attrib.get("damping", base_default_damping))
            old_arm = float(jnt_el.attrib.get("armature", base_default_armature))
            new_damp = max(old_damp, float(min_damping))
            new_arm = max(old_arm, float(min_armature))

            jnt_el.attrib["damping"] = f"{new_damp:.6g}"
            jnt_el.attrib["armature"] = f"{new_arm:.6g}"

            if (new_damp > old_damp) or (new_arm > old_arm):
                joints_updated.append(
                    {
                        "joint": jname,
                        "type": jtype,
                        "old_damping": old_damp,
                        "new_damping": new_damp,
                        "old_armature": old_arm,
                        "new_armature": new_arm,
                    }
                )

    # Collect wrap geoms referenced by spatial tendons and any group="2" wrap objects
    tendon_geom_sidesites: dict[str, list[str]] = {}
    for tgeom in root.findall(".//tendon/spatial/geom"):
        gname = tgeom.attrib.get("geom")
        if gname:
            tendon_geom_sidesites.setdefault(gname, [])
            ss = tgeom.attrib.get("sidesite")
            if ss and ss not in tendon_geom_sidesites[gname]:
                tendon_geom_sidesites[gname].append(ss)

    wrap_approximations: list[dict[str, Any]] = []
    if worldbody is not None:
        for geom_el in worldbody.findall(".//geom"):
            gname = geom_el.attrib.get("name", "")
            has_mesh = "mesh" in geom_el.attrib
            gtype = (
                geom_el.attrib.get("type", "mesh" if has_mesh else "sphere")
                .strip()
                .lower()
            )
            is_tendon_wrap = gname in tendon_geom_sidesites
            is_wrap_group = geom_el.attrib.get("group") == "2"

            if not (is_tendon_wrap or is_wrap_group) and gtype not in {"torus"}:
                continue

            if gtype in {"sphere", "cylinder", "plane", "mesh"}:
                continue

            orig_size = (
                _str2arr(geom_el.attrib["size"])
                if "size" in geom_el.attrib
                else np.array([0.01])
            )

            if gtype == "ellipsoid":
                orig_rot = _geom_rotation_matrix(geom_el)
                new_type, new_size, delta_rot = _fit_ellipsoid_to_primitive(
                    orig_size, aspect_threshold=ellipsoid_aspect_threshold
                )
                geom_el.attrib["type"] = new_type
                geom_el.attrib["size"] = _arr2str(new_size)
                if not np.allclose(delta_rot, np.eye(3)):
                    _set_geom_rotation_matrix(geom_el, orig_rot @ delta_rot)
                _adjust_sidesites_outside_primitive(
                    geom_el,
                    tendon_geom_sidesites.get(gname, []),
                    new_type,
                    new_size,
                )
                wrap_approximations.append(
                    {
                        "geom": gname,
                        "original_type": "ellipsoid",
                        "replacement_type": new_type,
                        "original_size": [float(v) for v in orig_size],
                        "replacement_size": [float(v) for v in new_size],
                        "referenced_by_tendon": is_tendon_wrap,
                    }
                )
            elif gtype == "torus":
                r_sphere = float(orig_size[0]) if orig_size.size >= 1 else 0.01
                new_size = np.array([max(r_sphere, 1e-5)])
                geom_el.attrib["type"] = "sphere"
                geom_el.attrib["size"] = _arr2str(new_size)
                wrap_approximations.append(
                    {
                        "geom": gname,
                        "original_type": "torus",
                        "replacement_type": "sphere",
                        "original_size": [float(v) for v in orig_size],
                        "replacement_size": [float(v) for v in new_size],
                        "referenced_by_tendon": is_tendon_wrap,
                    }
                )
            elif is_tendon_wrap:
                new_type, new_size, _ = _fit_ellipsoid_to_primitive(
                    orig_size, aspect_threshold=ellipsoid_aspect_threshold
                )
                geom_el.attrib["type"] = new_type
                geom_el.attrib["size"] = _arr2str(new_size)
                wrap_approximations.append(
                    {
                        "geom": gname,
                        "original_type": gtype,
                        "replacement_type": new_type,
                        "original_size": [float(v) for v in orig_size],
                        "replacement_size": [float(v) for v in new_size],
                        "referenced_by_tendon": True,
                    }
                )

    asset_el = root.find("asset")
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

    tree.write(str(out_path), pretty_print=True, xml_declaration=True, encoding="utf-8")

    # Compile with mujoco.MjModel.from_xml_path to verify numerical & MJX validity
    compiled = mujoco.MjModel.from_xml_path(str(out_path))
    data = mujoco.MjData(compiled)
    mujoco.mj_forward(compiled, data)
    if np.any(~np.isfinite(data.qacc)):
        raise RuntimeError(
            f"Non-finite accelerations encountered after MJX post-processing of {out_path}"
        )

    report = {
        "input_xml": str(in_path),
        "output_xml": str(out_path),
        "output_path": str(out_path),
        "damping_floor": float(min_damping),
        "armature_floor": float(min_armature),
        "joints_updated": joints_updated,
        "wrap_approximations": wrap_approximations,
        "solver_options": solver_options,
        "model_stats": {
            "nq": int(compiled.nq),
            "nv": int(compiled.nv),
            "nu": int(compiled.nu),
            "nbody": int(compiled.nbody),
            "ntendon": int(compiled.ntendon),
            "nwrap": int(compiled.nwrap),
        },
        "mjx_valid": True,
    }
    logger.info(
        f"MJX-ready model written to {out_path} "
        f"({len(wrap_approximations)} wrap approximations, {len(joints_updated)} joints floored)"
    )
    return report


def _build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Post-process a converted MuJoCo XML model for MJX / GPU RL simulation."
    )
    parser.add_argument("mjcf_file", type=str, help="Input MuJoCo .xml path")
    parser.add_argument(
        "--output_xml", type=str, default=None, help="Output MJX-ready .xml path"
    )
    parser.add_argument(
        "--min_damping", type=float, default=0.05, help="Minimum joint damping floor"
    )
    parser.add_argument(
        "--min_armature", type=float, default=0.005, help="Minimum joint armature floor"
    )
    parser.add_argument(
        "--integrator",
        type=str,
        default="implicitfast",
        choices=sorted(_VALID_MJX_INTEGRATORS),
        help="MuJoCo/MJX integrator",
    )
    parser.add_argument(
        "--iterations", type=int, default=20, help="Solver iterations"
    )
    parser.add_argument(
        "--ls_iterations", type=int, default=20, help="Line-search iterations"
    )
    parser.add_argument(
        "--timestep", type=float, default=None, help="Optional timestep override"
    )
    parser.add_argument(
        "--report_json", type=str, default=None, help="Optional path to save JSON report"
    )
    return parser


def main(argv: list[str] | None = None) -> dict[str, Any]:
    args = _build_cli_parser().parse_args(argv)
    report = make_mjx_ready(
        mjcf_file=args.mjcf_file,
        output_xml=args.output_xml,
        min_damping=args.min_damping,
        min_armature=args.min_armature,
        integrator=args.integrator,
        iterations=args.iterations,
        ls_iterations=args.ls_iterations,
        timestep=args.timestep,
    )
    if args.report_json is not None:
        Path(args.report_json).write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
    return report


if __name__ == "__main__":
    main()
