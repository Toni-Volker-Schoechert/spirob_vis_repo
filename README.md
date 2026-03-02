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
│   ├── spirob.urdf
│   └── meshes/         # STL meshes (optional; used when URDF contains <mesh> visuals)
├── src/
│   ├── app.py          # main entrypoint
│   ├── config.py       # defaults (overridable via CLI)
│   ├── rest_api.py     # FastAPI app + SharedState
│   ├── urdf_model.py   # URDF loading, FK, visual extraction, limits
│   └── viser_view.py   # viser scene + GUI toggles + pose updates
└── tools/
    └── demo_client.py  # sends joint states via REST
```

## Updating the robot visuals (Fusion 360 → STL → URDF)

This viewer can render **real robot geometry** via URDF `<mesh>` visuals (e.g. STL).
When the physical design changes, update the geometry by exporting new meshes and pointing the URDF to them.

### 1) Export STL meshes from Fusion 360 (one STL per segment/link)

**Goal:** each segment/link has a corresponding mesh file, exported in the **link-local coordinate frame**.

Recommended conventions:

- **One STL per URDF link** (e.g. `seg_0.stl`, `seg_1.stl`, …).
- Put all STL files in `Data/meshes/`.
- URDF uses **meters**, STL is unitless → if you model in **mm** (typical in Fusion), use `scale="0.001 0.001 0.001"` in URDF.

#### Fusion 360 workflow (robust)

1. Make sure each robot segment is a **separate Component** (e.g. `seg_0`, `seg_1`, …).
   Everything that is rigidly connected within a segment can be multiple bodies/components inside that segment.
2. Export in **local coordinates** (important):
   - Best results if the **joint axis center** is at `(0,0,0)` in the exported STL.
   - If you re-export in a new design, ensure the geometry is moved so the intended origin (e.g. joint axis) is at `(0,0,0)`
     before exporting.
3. Export:
   - Right-click the segment component → **Save as Mesh…** (German UI: *Als Netz speichern…*)
   - Format: **STL (Binary)**
   - Refinement: **Medium** (High only if needed)
   - Ensure it becomes **one file**, not “one file per body”
4. Name the file like the URDF link (example: link `seg_3` → `seg_3.stl`).

> Tip: If the mesh later shows a big offset/diagonal chain in the viewer, the STL likely contains a baked-in world position.
> Re-export so the part is in its local frame (joint center near origin).

### 2) Place meshes next to the URDF

Recommended layout:

```
Data/
  spirob.urdf
  meshes/
    seg_-1.stl
    seg_0.stl
    seg_1.stl
    ...
```

### 3) Update URDF visuals to use meshes

For each link, replace `<box>` visuals with `<mesh>`:

```xml
<link name="seg_0">
  <visual>
    <!-- If the mesh is already exported in the link frame, keep this at 0 -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="meshes/seg_0.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="seg0_color">
      <color rgba="1 0 0 1.0"/>
    </material>
  </visual>
</link>
```

**Important notes:**
- If your STL is exported in **mm**, keep `scale="0.001 0.001 0.001"`.
- If you export in meters, use `scale="1 1 1"`.
- If you aligned the STL to the link origin in Fusion, keep `<visual><origin ...>` at **zero** to avoid double offsets.

### 4) Joints define spacing (visual origin does NOT)

- The **distance between segments** is defined by the joint `<origin xyz="...">` values.
- If two segments overlap too much or too little, adjust the corresponding **joint origin**.

Example (spacing along Z):

```xml
<joint name="j_0" type="revolute">
  <parent link="seg_1"/>
  <child  link="seg_0"/>
  <origin xyz="0 0 0.018249" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  ...
</joint>
```

If you changed the physical segment length (CAD), you must update these distances accordingly.

