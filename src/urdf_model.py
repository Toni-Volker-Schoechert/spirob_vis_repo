"""urdf_model.py

Name: urdf_model.py
Date: 2026-02-15
Description:
    URDF loading + robot structure access.

    Responsibilities:
      - Load a URDF via `yourdfpy`.
      - Provide link/joint names.
      - Forward kinematics (base -> link transforms).
      - Extract simple visual geometry (currently URDF box visuals).
      - Parse joint limits from the URDF XML.

    This module intentionally contains *no visualization code*.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple
import xml.etree.ElementTree as ET

import numpy as np
from yourdfpy import URDF


def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    """Convert roll-pitch-yaw to a 3x3 rotation matrix.

    Parameters
    ----------
    rpy:
        Array-like of shape (3,) in radians: [roll, pitch, yaw].

    Returns
    -------
    np.ndarray
        Rotation matrix of shape (3, 3).
    """
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)

    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return (Rz @ Ry) @ Rx


def _origin_to_T(origin: Any | None) -> np.ndarray:
    """Convert a URDF <origin> (xyz/rpy) object to a 4x4 transform.

    Parameters
    ----------
    origin:
        yourdfpy origin object (has .xyz and .rpy) or None.

    Returns
    -------
    np.ndarray
        Homogeneous transform of shape (4, 4).
    """
    if origin is None or getattr(origin, "xyz", None) is None:
        xyz = np.array([0.0, 0.0, 0.0], dtype=float)
    else:
        xyz = np.asarray(origin.xyz, dtype=float)

    if origin is None or getattr(origin, "rpy", None) is None:
        rpy = np.array([0.0, 0.0, 0.0], dtype=float)
    else:
        rpy = np.asarray(origin.rpy, dtype=float)

    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


@dataclass(frozen=True)
class BoxVisual:
    """A simple URDF visual: box geometry in a link-local frame."""

    size: Tuple[float, float, float]
    origin_T: np.ndarray


class URDFModel:
    """Load and query a URDF robot model."""

    def __init__(self, urdf_path: str):
        """Create a URDFModel.

        Parameters
        ----------
        urdf_path:
            Path to a URDF file.
        """
        self.urdf_path = urdf_path
        self.robot = URDF.load(urdf_path)
        self._joint_limits = self._parse_joint_limits_from_xml(urdf_path)

    @property
    def link_names(self) -> List[str]:
        """List of all link names in the URDF."""
        return list(self.robot.link_map.keys())

    @property
    def joint_names(self) -> List[str]:
        """List of all joint names in the URDF (including fixed joints)."""
        return list(self.robot.joint_names)

    def joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """Return joint limits for actuated joints.

        Returns
        -------
        dict
            Mapping joint_name -> (lower_rad, upper_rad). Only joints with an
            explicit <limit lower=... upper=...> entry are returned.
        """
        return dict(self._joint_limits)

    @staticmethod
    def _parse_joint_limits_from_xml(urdf_path: str) -> Dict[str, Tuple[float, float]]:
        """Parse joint limits from the URDF XML.

        Notes
        -----
        URDF uses radians for revolute joint limits.

        Returns
        -------
        dict
            Mapping joint_name -> (lower_rad, upper_rad).
        """
        root = ET.parse(urdf_path).getroot()
        out: Dict[str, Tuple[float, float]] = {}
        for j in root.findall("joint"):
            jtype = j.attrib.get("type", "")
            if jtype == "fixed":
                continue
            name = j.attrib.get("name")
            if not name:
                continue
            lim = j.find("limit")
            if lim is None:
                continue
            try:
                lo = float(lim.attrib.get("lower"))
                hi = float(lim.attrib.get("upper"))
            except (TypeError, ValueError):
                continue
            if np.isfinite(lo) and np.isfinite(hi):
                out[name] = (lo, hi)
        return out

    def forward_kinematics(self, joint_values: Dict[str, float]) -> Dict[str, np.ndarray]:
        """Compute forward kinematics for all links.

        Parameters
        ----------
        joint_values:
            Mapping joint_name -> value (radians for revolute joints).

        Returns
        -------
        dict
            Mapping link_name -> 4x4 transform (base -> link).
        """
        self.robot.update_cfg(joint_values)
        return {ln: self.robot.get_transform(frame_to=ln) for ln in self.link_names}

    def extract_visuals(self) -> Dict[str, List[Dict[str, Any]]]:
        """Extract URDF visual geometries.

        Currently supported:
          - <visual><geometry><box size="sx sy sz"/></geometry></visual>

        Returns
        -------
        dict
            Mapping link_name -> list of visual dicts.

            Each visual dict contains:
              - type: "box"
              - size: (sx, sy, sz)
              - origin_T: 4x4 transform of the visual in the link frame
        """
        out: Dict[str, List[Dict[str, Any]]] = {}

        # yourdfpy does not expose a stable `robot.links` attribute across versions;
        # `link_map` is stable.
        for link in self.robot.link_map.values():
            visuals: List[Dict[str, Any]] = []
            link_visuals = getattr(link, "visuals", None)
            if link_visuals is None:
                # Some versions use `visual` instead of `visuals`.
                v_single = getattr(link, "visual", None)
                link_visuals = [] if v_single is None else [v_single]

            for vis in link_visuals:
                geom = getattr(vis, "geometry", None)
                if geom is None:
                    continue
                box = getattr(geom, "box", None)
                if box is None:
                    continue

                size = tuple(float(x) for x in box.size)
                origin_T = _origin_to_T(getattr(vis, "origin", None))
                visuals.append({"type": "box", "size": size, "origin_T": origin_T})

            out[getattr(link, "name", "")] = visuals

        return out
