"""app.py

Name: app.py
Date: 2026-02-15
Description:
    Main application entrypoint.

    This script:
      - loads the URDF model
      - starts a viser server for visualization
      - optionally starts a REST API (FastAPI) in a background thread
      - provides GUI sliders (in degrees) and a dropdown to select the input source
        (Sliders vs. REST)

    Run:
      python src/app.py --urdf Data/spirob.urdf

    See README.md for details.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import threading
import time
from typing import Dict, Tuple

import numpy as np
import uvicorn

from config import Config
from rest_api import SharedState, create_app
from urdf_model import URDFModel
from viser_view import ViserView


def _joint_key(name: str) -> int:
    """Natural sort key for joint names like 'j_0', 'j_10', ..."""
    try:
        return int(name.split("_")[-1])
    except Exception:
        return 10**9


def parse_args_to_config() -> Config:
    """Parse CLI args and return a Config.

    The returned config starts from Config() defaults and is overridden by
    explicitly provided CLI arguments.

    Returns
    -------
    Config
        Final configuration.
    """
    cfg = Config()

    p = argparse.ArgumentParser(description="Spirob viser visualization")

    p.add_argument("--urdf", dest="urdf_path", default=cfg.urdf_path, help="Path to URDF file")
    p.add_argument("--fps", type=int, default=cfg.fps, help="Render/update FPS")
    p.add_argument("--root-path", default=cfg.root_path, help="Viser scene root path")

    p.add_argument("--api-host", default=cfg.api_host, help="REST API host")
    p.add_argument("--api-port", type=int, default=cfg.api_port, help="REST API port")

    p.add_argument(
        "--grid",
        dest="show_grid",
        action=argparse.BooleanOptionalAction,
        default=cfg.show_grid,
        help="Show grid (use --grid / --no-grid)",
    )
    p.add_argument(
        "--base-frame",
        dest="show_base_frame",
        action=argparse.BooleanOptionalAction,
        default=cfg.show_base_frame,
        help="Show base frame (use --base-frame / --no-base-frame)",
    )

    p.add_argument(
        "--link-frames",
        dest="show_link_frames",
        action=argparse.BooleanOptionalAction,
        default=cfg.show_link_frames,
        help="Show per-link debug frames (use --link-frames / --no-link-frames)",
    )

    args = p.parse_args()
    return replace(cfg, **vars(args))


def _start_rest_api(shared: SharedState, host: str, port: int) -> None:
    """Run the REST API server (blocking).

    Intended to be used as a thread target.

    Parameters
    ----------
    shared:
        SharedState instance that the API reads/writes.
    host:
        Bind address (e.g. "0.0.0.0").
    port:
        Port to bind (e.g. 8000).
    """
    app = create_app(shared)
    uv_cfg = uvicorn.Config(app, host=host, port=port, log_level="info")
    uvicorn.Server(uv_cfg).run()


def _add_joint_sliders(server, joint_limits_rad: Dict[str, Tuple[float, float]]):
    """Create GUI sliders for each actuated joint.

    Sliders are shown in degrees for usability.

    Parameters
    ----------
    server:
        Viser server instance (view.server).
    joint_limits_rad:
        Mapping joint_name -> (lower_rad, upper_rad).

    Returns
    -------
    dict
        Mapping joint_name -> slider handle.
    """
    sliders = {}
    with server.gui.add_folder("Joints (deg)"):
        for jn, (lo, hi) in sorted(joint_limits_rad.items(), key=lambda kv: _joint_key(kv[0])):
            s = server.gui.add_slider(
                label=jn,
                min=float(np.degrees(lo)),
                max=float(np.degrees(hi)),
                step=0.5,
                initial_value=0.0,
            )
            sliders[jn] = s
    return sliders


def run(cfg: Config) -> None:
    """Run the visualization app."""

    model = URDFModel(cfg.urdf_path)
    view = ViserView(cfg)

    # Scene + GUI
    view.setup_reference_scene()
    view.create_link_frames(model.link_names)

    link_visuals = model.extract_visuals()
    view.create_link_visuals(link_visuals)

    view.add_scene_toggles()

    # Joint controls
    joint_limits = model.joint_limits()  # actuated joints only
    sliders = _add_joint_sliders(view.server, joint_limits)

    input_source = view.server.gui.add_dropdown(
        "Input source", options=["Sliders", "REST"], initial_value="Sliders"
    )

    # REST API
    shared = SharedState()
    shared.limits = joint_limits
    shared.allowed_joints = set(joint_limits.keys())

    api_thread = threading.Thread(
        target=_start_rest_api,
        args=(shared, cfg.api_host, cfg.api_port),
        daemon=True,
    )
    api_thread.start()

    # Render loop state (radians)
    joint_values_rad: Dict[str, float] = {jn: 0.0 for jn in joint_limits.keys()}

    print("\n--- Spirob Visualizer ---")
    print("Viser: open the URL printed by viser in your terminal/browser.")
    print(f"REST API: http://{cfg.api_host}:{cfg.api_port}")
    print("Endpoints: GET /joints, POST /state")

    while True:
        if input_source.value == "Sliders":
            for jn, s in sliders.items():
                joint_values_rad[jn] = float(np.deg2rad(s.value))
        else:
            with shared.lock:
                for jn, v in shared.latest_rad.items():
                    joint_values_rad[jn] = float(v)

        poses = model.forward_kinematics(joint_values_rad)
        view.update_link_poses(poses)

        time.sleep(1.0 / max(1, int(cfg.fps)))


if __name__ == "__main__":
    run(parse_args_to_config())
