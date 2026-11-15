"""Contains the ``CoordinateLimitForce`` parser.

Converts OpenSim ``CoordinateLimitForce`` passive joint-stop elements into MuJoCo
joint limit impedance (``limited``, ``range``, ``damping``, ``solimplimit``,
``solreflimit``) for both rotational (``hinge``) and translational (``slide``)
coordinates.
"""

from __future__ import annotations

import math

import numpy as np
from loguru import logger

from myoconverter.xml import config as cfg
from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2vec


def _parse_optional_bool(xml_el, tag: str, default: bool) -> bool:
    child = xml_el.find(tag)
    if child is None or child.text is None:
        return default
    txt = child.text.strip().lower()
    if txt in {"true", "1", "yes", "t"}:
        return True
    if txt in {"false", "0", "no", "f"}:
        return False
    return default


def _parse_optional_float(xml_el, tag: str, default: float) -> float:
    child = xml_el.find(tag)
    if child is None or child.text is None or not child.text.strip():
        return default
    return float(child.text.strip())


def _find_mjcf_root():
    for attr in ("M_WORLDBODY", "MUJOCO", "C_MJCF"):
        el = getattr(cfg, attr, None)
        if el is not None:
            return el
    return None


def _get_default_joint_attrib(
    attr_name: str, fallback: str | None = None
) -> str | None:
    mujoco_root = getattr(cfg, "MUJOCO", None)
    if mujoco_root is None:
        mjcf_root = _find_mjcf_root()
        if mjcf_root is not None:
            mujoco_root = mjcf_root.getroottree().getroot()
    if mujoco_root is not None and mujoco_root.tag == "mujoco":
        default_jnt = mujoco_root.find("default/joint")
        if default_jnt is not None and attr_name in default_jnt.attrib:
            return default_jnt.attrib[attr_name]
    return fallback


class CoordinateLimitForce(IParser):
    """Parses and converts an OpenSim ``CoordinateLimitForce`` XML element to MuJoCo.

    Unit conventions in OpenSim ``CoordinateLimitForce``:
    - Rotational coordinates (``hinge`` joints in MuJoCo):
      * ``lower_limit``, ``upper_limit``, ``transition`` are specified in degrees
        and converted to radians via ``* pi / 180``.
      * ``lower_stiffness``, ``upper_stiffness`` (``N*m/deg``) and ``damping``
        (``N*m*s/deg``) are converted to SI rotational impedance (``N*m/rad`` and
        ``N*m*s/rad``) via ``* 180 / pi`` (since 1 rad = 180/pi deg, a 1 deg
        excursion equals pi/180 rad and produces ``(K_deg * 180/pi) * (pi/180) = K_deg`` N*m).
    - Translational coordinates (``slide`` joints in MuJoCo):
      * ``lower_limit``, ``upper_limit``, ``transition`` remain in meters (``m``).
      * ``lower_stiffness``, ``upper_stiffness`` remain in ``N/m`` and ``damping`` in ``N*s/m``.

    MuJoCo soft joint-limit impedance parameterization:
    - When effective stiffness ``k > 0``, ``solreflimit`` uses MuJoCo's direct negative
      stiffness/damping parameterization ``[-k, -b]`` and ``solimplimit`` encodes the
      smooth transition zone ``[0.0001, 0.99, max(transition, 1e-4), 0.5, 1]``.
    """

    def parse(self, xml):
        """Parse an OpenSim ``CoordinateLimitForce`` element and configure the target MuJoCo joint.

        :param xml: OpenSim ``CoordinateLimitForce`` XML element
        :return: None
        """
        force_name = xml.attrib.get("name", "<unnamed>")

        # Respect <appliesForce> (false -> skip) and <isDisabled> (true -> skip)
        if not _parse_optional_bool(xml, "appliesForce", True):
            logger.info(
                f"Skipping CoordinateLimitForce '{force_name}' because appliesForce is false."
            )
            return
        if _parse_optional_bool(xml, "isDisabled", False):
            logger.info(
                f"Skipping CoordinateLimitForce '{force_name}' because isDisabled is true."
            )
            return

        coord_el = xml.find("coordinate")
        if coord_el is None or coord_el.text is None or not coord_el.text.strip():
            logger.warning(
                f"CoordinateLimitForce '{force_name}' is missing <coordinate>; skipping."
            )
            return
        coord_raw = coord_el.text.strip()

        mjcf_root = _find_mjcf_root()
        if mjcf_root is None:
            logger.warning(
                f"MuJoCo XML tree is not initialized while parsing CoordinateLimitForce '{force_name}'; skipping."
            )
            return

        joint = mjcf_root.find(f".//joint[@name='{coord_raw}']")
        if joint is None and "/" in coord_raw:
            coord_short = coord_raw.rsplit("/", 1)[-1]
            joint = mjcf_root.find(f".//joint[@name='{coord_short}']")
        if joint is None:
            logger.warning(
                f"Target joint '{coord_raw}' not found in MuJoCo XML for CoordinateLimitForce '{force_name}'; skipping."
            )
            return

        jnt_type = joint.attrib.get("type", "hinge").strip().lower()
        if jnt_type not in {"hinge", "slide"}:
            logger.warning(
                f"Unsupported joint type '{jnt_type}' on joint '{joint.attrib.get('name')}' "
                f"for CoordinateLimitForce '{force_name}'; skipping."
            )
            return

        lower_limit = _parse_optional_float(xml, "lower_limit", -math.inf)
        upper_limit = _parse_optional_float(xml, "upper_limit", math.inf)
        lower_stiffness = _parse_optional_float(xml, "lower_stiffness", 0.0)
        upper_stiffness = _parse_optional_float(xml, "upper_stiffness", 0.0)
        damping = _parse_optional_float(xml, "damping", 0.0)
        transition = abs(_parse_optional_float(xml, "transition", 0.0))

        if jnt_type == "hinge":
            deg2rad = math.pi / 180.0
            rad2deg = 180.0 / math.pi
            lower_limit *= deg2rad
            upper_limit *= deg2rad
            transition *= deg2rad
            lower_stiffness *= rad2deg
            upper_stiffness *= rad2deg
            damping_si = damping * rad2deg
        else:
            damping_si = damping

        if (
            math.isfinite(lower_limit)
            and math.isfinite(upper_limit)
            and lower_limit > upper_limit
        ):
            logger.warning(
                f"CoordinateLimitForce '{force_name}' has lower_limit > upper_limit; swapping bounds."
            )
            lower_limit, upper_limit = upper_limit, lower_limit
            lower_stiffness, upper_stiffness = upper_stiffness, lower_stiffness

        pos_stiffnesses = [
            k for k in (lower_stiffness, upper_stiffness) if k > 0.0 and math.isfinite(k)
        ]
        stiffness = float(np.mean(pos_stiffnesses)) if pos_stiffnesses else 0.0

        default_limited = _get_default_joint_attrib(
            "limited", "true" if "range" in joint.attrib else "false"
        )
        was_limited = (
            joint.attrib.get("limited", default_limited).strip().lower() == "true"
        )
        has_existing_range = "range" in joint.attrib

        has_finite_clf_bounds = math.isfinite(lower_limit) or math.isfinite(upper_limit)
        if has_finite_clf_bounds and (stiffness > 0.0 or not was_limited):
            if was_limited and has_existing_range:
                cur_low, cur_high = [float(v) for v in str2vec(joint.attrib["range"])]
                use_low = math.isfinite(lower_limit) and (
                    lower_stiffness > 0.0 or upper_stiffness <= 0.0
                )
                use_high = math.isfinite(upper_limit) and (
                    upper_stiffness > 0.0 or lower_stiffness <= 0.0
                )
                new_low = max(cur_low, lower_limit) if use_low else cur_low
                new_high = min(cur_high, upper_limit) if use_high else cur_high
                if new_low >= new_high:
                    if (
                        math.isfinite(lower_limit)
                        and math.isfinite(upper_limit)
                        and lower_limit < upper_limit
                    ):
                        new_low, new_high = lower_limit, upper_limit
                    else:
                        mid = 0.5 * (new_low + new_high)
                        new_low, new_high = mid - 1e-5, mid + 1e-5
            else:
                if has_existing_range:
                    cur_low, cur_high = [
                        float(v) for v in str2vec(joint.attrib["range"])
                    ]
                else:
                    cur_low, cur_high = -math.pi, math.pi
                new_low = lower_limit if math.isfinite(lower_limit) else cur_low
                new_high = upper_limit if math.isfinite(upper_limit) else cur_high
                if new_low >= new_high:
                    mid = 0.5 * (new_low + new_high)
                    new_low, new_high = mid - 1e-5, mid + 1e-5

            joint.attrib["limited"] = "true"
            joint.attrib["range"] = f"{new_low:.6g} {new_high:.6g}"

        if "damping" in joint.attrib:
            existing_damping = float(joint.attrib["damping"])
        else:
            default_damp_str = _get_default_joint_attrib("damping", "0")
            existing_damping = (
                float(default_damp_str) if default_damp_str is not None else 0.0
            )

        added_damping = (
            max(float(damping_si), 0.0) if math.isfinite(damping_si) else 0.0
        )
        if added_damping > 0.0 or "damping" in joint.attrib:
            total_damping = existing_damping + added_damping
            joint.attrib["damping"] = f"{total_damping:.6g}"

        trans_width = max(
            float(transition) if math.isfinite(transition) else 0.0, 1e-4
        )
        if stiffness > 0.0:
            damping_limit = added_damping
            if "solreflimit" in joint.attrib:
                prev_solref = str2vec(joint.attrib["solreflimit"])
                if len(prev_solref) == 2 and prev_solref[0] < 0.0:
                    stiffness = max(stiffness, -float(prev_solref[0]))
                    damping_limit = max(damping_limit, -float(prev_solref[1]))

            neg_k = -stiffness
            neg_b = -damping_limit if damping_limit > 0.0 else 0.0
            joint.attrib["solimplimit"] = f"0.0001 0.99 {trans_width:.6g} 0.5 1"
            joint.attrib["solreflimit"] = f"{neg_k:.6g} {neg_b:.6g}"
