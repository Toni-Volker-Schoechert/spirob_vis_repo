"""urdf_model.py

Name: urdf_model.py
Date: 2026-02-15
Description:
    URDF loading + robot structure access.

    Responsibilities:
      - Load a URDF via `yourdfpy`.
      - Provide link/joint names.
      - Forward kinematics (base -> link transforms).
      - Extract real visuals from urdf
      - Parse joint limits from the URDF XML.

    This module intentionally contains *no visualization code*.
"""

from __future__ import annotations

import os
from pathlib import Path

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

def _origin_xml_to_T(origin_el: ET.Element | None) -> np.ndarray:
    """Convert URDF XML <origin xyz=".." rpy=".."> to 4x4 transform."""
    xyz = np.array([0.0, 0.0, 0.0], dtype=float)
    rpy = np.array([0.0, 0.0, 0.0], dtype=float)

    if origin_el is not None:
        if "xyz" in origin_el.attrib:
            parts = origin_el.attrib["xyz"].replace(",", " ").split()
            if len(parts) >= 3:
                xyz = np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)
        if "rpy" in origin_el.attrib:
            parts = origin_el.attrib["rpy"].replace(",", " ").split()
            if len(parts) >= 3:
                rpy = np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)

    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def _parse_rgba(rgba_str: str | None) -> tuple[float, float, float, float] | None:
    if not rgba_str:
        return None
    parts = rgba_str.replace(",", " ").split()
    if len(parts) < 3:
        return None
    r = float(parts[0]); g = float(parts[1]); b = float(parts[2])
    a = float(parts[3]) if len(parts) >= 4 else 1.0
    return (r, g, b, a)

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
        self._urdf_dir = Path(urdf_path).resolve().parent
        self._xml_root = ET.parse(urdf_path).getroot()

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

    def _resolve_mesh_path(self, filename: str) -> str:
        # Handle file:// and package:// in a simple local way.
        fn = filename.strip()
        if fn.startswith("file://"):
            fn = fn[len("file://"):]
        elif fn.startswith("package://"):
            fn = fn[len("package://"):]  # treat as relative under URDF folder

        p = Path(fn)
        if p.is_absolute():
            return str(p)
        return str((self._urdf_dir / p).resolve())

    def extract_visuals(self) -> Dict[str, List[Dict[str, Any]]]:
        """Extract URDF visuals from XML: box + mesh (+ material rgba)."""
        out: Dict[str, List[Dict[str, Any]]] = {}

        for link_el in self._xml_root.findall("link"):
            link_name = link_el.attrib.get("name", "")
            visuals: List[Dict[str, Any]] = []

            for vis_el in link_el.findall("visual"):
                origin_T = _origin_xml_to_T(vis_el.find("origin"))

                # material / color
                rgba = None
                mat_el = vis_el.find("material")
                if mat_el is not None:
                    color_el = mat_el.find("color")
                    if color_el is not None:
                        rgba = _parse_rgba(color_el.attrib.get("rgba"))

                geom_el = vis_el.find("geometry")
                if geom_el is None:
                    continue

                box_el = geom_el.find("box")
                if box_el is not None and "size" in box_el.attrib:
                    parts = box_el.attrib["size"].replace(",", " ").split()
                    if len(parts) >= 3:
                        size = (float(parts[0]), float(parts[1]), float(parts[2]))
                        visuals.append(
                            {"type": "box", "size": size, "origin_T": origin_T, "rgba": rgba}
                        )
                    continue

                mesh_el = geom_el.find("mesh")
                if mesh_el is not None:
                    filename = mesh_el.attrib.get("filename") or mesh_el.attrib.get("url")
                    if not filename:
                        continue
                    scale_attr = mesh_el.attrib.get("scale", "1 1 1").replace(",", " ").split()
                    if len(scale_attr) >= 3:
                        scale = (float(scale_attr[0]), float(scale_attr[1]), float(scale_attr[2]))
                    else:
                        scale = (1.0, 1.0, 1.0)

                    visuals.append(
                        {
                            "type": "mesh",
                            "path": self._resolve_mesh_path(filename),
                            "scale": scale,
                            "origin_T": origin_T,
                            "rgba": rgba,
                        }
                    )
                    continue

            out[link_name] = visuals

        return out
