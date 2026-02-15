"""rest_api.py

Name: rest_api.py
Date: 2026-02-15
Description:
    Minimal REST API to feed joint states into the viser visualization.

    This module provides:
      - SharedState: thread-safe container for the latest joint state.
      - create_app(): FastAPI app with endpoints:
          GET  /health
          GET  /joints
          GET  /state
          POST /state

    Units
    -----
    - URDF joint limits are in radians.
    - The API accepts joint values in radians or degrees (payload field `unit`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple
import time
import threading

import numpy as np
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel


def _joint_key(name: str) -> int:
    """Natural sort key for joint names like 'j_0', 'j_10', ..."""
    try:
        return int(name.split("_")[-1])
    except Exception:
        return 10**9


@dataclass
class SharedState:
    """Thread-safe shared state between REST thread and render loop."""

    # Latest values (radians): joint -> rad
    latest_rad: Dict[str, float] = field(default_factory=dict)

    # Timestamp (time.time()) of last update
    updated_at: float = 0.0

    # Mutex
    lock: threading.Lock = field(default_factory=threading.Lock)

    # Limits: joint -> (lower_rad, upper_rad)
    limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)

    # Allowed joints (names)
    allowed_joints: set[str] = field(default_factory=set)


class StateIn(BaseModel):
    """Incoming state payload."""

    unit: str = "rad"  # "rad" or "deg"
    joints: Dict[str, float]


def create_app(shared: SharedState) -> FastAPI:
    """Create a FastAPI application.

    Parameters
    ----------
    shared:
        SharedState instance. The app reads/writes this object.

    Returns
    -------
    FastAPI
        The initialized FastAPI app.
    """

    app = FastAPI(title="Spirob Visualizer API", version="0.2")

    @app.get("/health")
    def health() -> Dict[str, float | bool]:
        """Basic health check."""
        return {"ok": True, "time": time.time()}

    @app.get("/joints")
    def joints() -> Dict[str, object]:
        """List actuated joints and their limits."""
        joint_list = sorted(list(shared.allowed_joints), key=_joint_key)
        limits_sorted = {
            k: [float(shared.limits[k][0]), float(shared.limits[k][1])]
            for k in joint_list
            if k in shared.limits
        }
        return {"joints": joint_list, "limits_rad": limits_sorted}

    @app.get("/state")
    def get_state() -> Dict[str, object]:
        """Get latest stored joint values (radians)."""
        with shared.lock:
            return {"updated_at": shared.updated_at, "joints_rad": dict(shared.latest_rad)}

    @app.post("/state")
    def set_state(payload: StateIn) -> Dict[str, object]:
        """Set (partial) joint state.

        Notes
        -----
        - You may send only a subset of joints.
        - Values are clamped to known URDF limits.
        """
        unit = payload.unit.lower().strip()
        if unit not in ("rad", "deg"):
            raise HTTPException(status_code=400, detail="unit must be 'rad' or 'deg'")

        joints_rad: Dict[str, float] = {}
        for j, v in payload.joints.items():
            if j not in shared.allowed_joints:
                raise HTTPException(status_code=400, detail=f"unknown joint: {j}")

            vr = float(v)
            if unit == "deg":
                vr = float(np.deg2rad(vr))

            if j in shared.limits:
                lo, hi = shared.limits[j]
                vr = float(np.clip(vr, lo, hi))

            joints_rad[j] = vr

        with shared.lock:
            shared.latest_rad.update(joints_rad)
            shared.updated_at = time.time()

        return {"ok": True, "updated_at": shared.updated_at, "received": sorted(list(joints_rad.keys()), key=_joint_key)}

    return app
