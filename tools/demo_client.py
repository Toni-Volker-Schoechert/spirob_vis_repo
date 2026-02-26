"""demo_client.py

Name: demo_client.py
Date: 2026-02-15
Description:
    Small client script to demonstrate how to feed joint data into the
    visualization via the REST API.

    Two variants are shown:

    1) stream_state_frames():
       Sends joint values continuously to POST /state (e.g. 50 Hz).
       -> Works with the current API in this repo.

    2) send_trajectory_and_play():
       Sends an entire trajectory to POST /trajectory.
       -> This requires an additional /trajectory endpoint (not implemented in
          the current server). The function is included as a template.

    Usage:
      python tools/demo_client.py --base-url http://127.0.0.1:8000
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Dict, List

import requests


def stream_state_frames(
    base_url: str,
    duration_s: float = 20.0,
    hz: float = 50.0,
    unit: str = "deg",
) -> None:
    """Variant 1: Stream joint states at a fixed rate.

    Parameters
    ----------
    base_url:
        REST base URL, e.g. "http://127.0.0.1:8000".
    duration_s:
        How long to send data.
    hz:
        Send frequency.
    unit:
        "deg" or "rad".

    Notes
    -----
    This is the easiest option for live visualization.
    """

    url = f"{base_url.rstrip('/')}/state"
    dt = 1.0 / hz

    print(f"[stream_state_frames] Sending for {duration_s:.1f}s at {hz:.1f} Hz to {url}")

    t0 = time.time()
    k = 0
    while True:
        t = time.time() - t0
        if t >= duration_s:
            break

        # Example signal (adjust joint names for your robot):
        j10 = 29.0 * math.sin(2.0 * math.pi * 0.3 * t)  # +/-15 deg
        j5 = -29.0 * math.sin(2.0 * math.pi * 0.5 * t + 0.7)

        payload = {"unit": unit, "joints": {"j_10": j10, "j_5": j5}}
        r = requests.post(url, json=payload, timeout=0.5)
        if r.status_code != 200:
            raise RuntimeError(f"Server error {r.status_code}: {r.text}")

        k += 1
        sleep_for = (t0 + k * dt) - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)

    print("[stream_state_frames] done")


def send_trajectory_and_play(
    base_url: str,
    duration_s: float = 10.0,
    hz: float = 50.0,
    unit: str = "deg",
) -> None:
    """Variant 2: Send a whole trajectory (template).

    Parameters
    ----------
    base_url:
        REST base URL.
    duration_s:
        Trajectory length.
    hz:
        Sample rate.
    unit:
        "deg" or "rad".

    Notes
    -----
    This function expects a server endpoint POST /trajectory, which is not part
    of the current repo. Keep it as a starting point if you add this feature.
    """

    url = f"{base_url.rstrip('/')}/trajectory"
    dt = 1.0 / hz
    n = int(duration_s * hz)

    frames: List[Dict[str, float]] = []
    for i in range(n):
        t = i * dt
        j10 = 15.0 * math.sin(2.0 * math.pi * 0.3 * t)
        j5 = 10.0 * math.sin(2.0 * math.pi * 0.5 * t + 0.7)
        frames.append({"j_10": j10, "j_5": j5})

    payload = {"unit": unit, "dt": dt, "frames": frames, "loop": False, "start": True}
    r = requests.post(url, json=payload, timeout=10.0)
    if r.status_code != 200:
        raise RuntimeError(f"Server error {r.status_code}: {r.text}")

    print("[send_trajectory_and_play] response:", r.json())


def main() -> None:
    p = argparse.ArgumentParser(description="Demo client for Spirob Visualizer REST API")
    p.add_argument("--base-url", default="http://127.0.0.1:8000", help="REST base URL")
    p.add_argument("--duration", type=float, default=8.0)
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--unit", choices=["deg", "rad"], default="deg")
    args = p.parse_args()

    stream_state_frames(args.base_url, args.duration, args.hz, args.unit)


if __name__ == "__main__":
    main()
