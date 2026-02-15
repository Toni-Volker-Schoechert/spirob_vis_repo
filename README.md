# Spirob Visualizer (viser + URDF + REST)

This repository provides a small, practical viewer for a URDF robot model.
It is designed for **live visualization** of joint angles:

- **GUI sliders** (degrees) to move joints manually
- **REST API** to stream joint angles from an external process (e.g. simulation/controller)
- Reference visuals (**grid + base coordinate frame**) with GUI toggles
- Optional per-link debug frames (GUI toggle, default off)

## Quick start

### 1) Install

Create and activate a venv, then:

```bash
pip install -r requirements.txt
```

### 2) Run the viewer

From the repo root:

```bash
python src/app.py --urdf Data/spirob.urdf
```

You will see a URL printed by `viser`. Open it in your browser.

## CLI parameters

Common parameters:

```bash
python src/app.py \
  --urdf Data/spirob.urdf \
  --fps 60 \
  --api-host 0.0.0.0 \
  --api-port 8000 \
  --root-path /Visualisierung \
  --no-grid \
  --no-base-frame \
  --link-frames
```

Notes:
- `--no-grid` and `--no-base-frame` only hide the **reference visuals**.
- The robot visuals stay visible unless you disable them in the GUI.

## REST API

The viewer starts a REST API server in a background thread.

### Endpoints

- `GET /health`
- `GET /joints` → joint names + limits (radians)
- `GET /state` → latest stored joint state
- `POST /state` → set (partial) joint state

### Example: list joints

```bash
curl http://127.0.0.1:8000/joints
```

### Example: set joint angles (degrees)

```bash
curl -X POST http://127.0.0.1:8000/state \
  -H "Content-Type: application/json" \
  -d '{"unit":"deg","joints":{"j_0":10,"j_1":-5,"j_2":15}}'
```

### Example: set joint angles (radians)

```bash
curl -X POST http://127.0.0.1:8000/state \
  -H "Content-Type: application/json" \
  -d '{"unit":"rad","joints":{"j_0":0.2,"j_1":-0.1}}'
```

### Input source switching

In the viser GUI there is a dropdown `Input source`:
- **Sliders**: use GUI sliders (degrees)
- **REST**: use the latest state received via REST

## Demo client

A minimal streaming client is included:

```bash
python tools/demo_client.py --base-url http://127.0.0.1:8000
```

This sends a simple sinusoidal motion at ~50 Hz via `POST /state`.

## Project structure

```
.
├── Data/
│   └── spirob.urdf
├── src/
│   ├── app.py          # main entrypoint
│   ├── config.py       # defaults (overridable via CLI)
│   ├── rest_api.py     # FastAPI app + SharedState
│   ├── urdf_model.py   # URDF loading, FK, visual extraction, limits
│   └── viser_view.py   # viser scene + GUI toggles + pose updates
└── tools/
    └── demo_client.py  # sends joint states via REST
```

## Notes / limitations

- Visual extraction currently supports **URDF box visuals**. If your URDF uses mesh visuals, you can either:
  1) extend `URDFModel.extract_visuals()` + `ViserView.create_link_visuals()` to load meshes, or
  2) switch to `viser.extras.ViserUrdf` (viser provides a URDF helper that can load meshes).

- Joint limits are parsed from URDF XML and assumed to be **radians** (URDF standard).
