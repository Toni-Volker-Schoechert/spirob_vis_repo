"""config.py

Name: config.py
Date: 2026-02-15
Description:
    Default configuration for the Spirob viser visualization.

    This module contains *defaults only* (no side effects). Runtime overrides are
    handled in `app.py` via argparse.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    """Configuration values (defaults) for the visualization app."""

    # --- Paths ---
    urdf_path: str = "Data/spirob.urdf"

    # Root path in viser scene tree. Everything this app creates lives under it.
    root_path: str = "/Visualisierung"

    # --- Runtime ---
    fps: int = 60

    # --- REST API ---
    api_host: str = "0.0.0.0"
    api_port: int = 8000

    # --- Reference visuals (initial GUI state) ---
    show_grid: bool = True
    grid_width: float = 2.0
    grid_height: float = 2.0

    show_base_frame: bool = True
    base_axes_length: float = 0.5
    base_axes_radius: float = 0.002
    base_origin_radius: float = 0.01

    # --- Debug visuals (initial GUI state) ---
    # Small coordinate frames on every link. Useful for debugging, usually clutter.
    show_link_frames: bool = False
    show_link_axes: bool = True

    # --- Robot visuals (initial GUI state) ---
    show_robot_visuals: bool = True
