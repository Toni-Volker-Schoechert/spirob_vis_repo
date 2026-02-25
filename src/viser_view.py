"""viser_view.py

Name: viser_view.py
Date: 2026-02-15
Description:
    Viser visualization layer for the Spirob URDF viewer.

    Responsibilities:
      - Create the Viser server.
      - Create reference visuals (grid + base frame) and GUI toggles.
      - Create optional per-link debug frames (toggle, default off).
      - Create robot visuals
      - Update poses every frame.

    Notes
    -----
    This module is intentionally a "thin view" layer: it does not perform any
    kinematics; it only consumes poses from URDFModel.forward_kinematics().
"""

from __future__ import annotations

from typing import Dict, List, Optional

import viser
import viser.transforms as vt
import numpy as np
import trimesh


class ViserView:
    """Viser visualization wrapper."""

    def __init__(self, cfg):
        """Create a ViserView.

        Parameters
        ----------
        cfg:
            Config instance.
        """
        self.cfg = cfg
        self.server = viser.ViserServer()

        # Reference visuals
        self._grid: Optional[viser.SceneNodeHandle] = None
        self._base_frame: Optional[viser.SceneNodeHandle] = None

        # Debug link frames
        self.link_frames: Dict[str, viser.SceneNodeHandle] = {}

        # Robot visuals (per link: list of handles)
        self.link_visuals: Dict[str, List[viser.SceneNodeHandle]] = {}
        self._mesh_cache: Dict[str, trimesh.Trimesh] = {}

        # GUI checkboxes (stored so we can query their state if needed)
        self._cb_grid = None
        self._cb_base = None
        self._cb_link_frames = None
        self._cb_robot_visuals = None

    # ---- scene paths -------------------------------------------------

    @property
    def _ref_root(self) -> str:
        return f"{self.cfg.root_path}/ref"

    @property
    def _robot_root(self) -> str:
        return f"{self.cfg.root_path}/robot"

    @property
    def _debug_root(self) -> str:
        return f"{self.cfg.root_path}/debug"

    # ---- scene creation ----------------------------------------------

    def setup_reference_scene(self) -> None:
        """Create grid + base frame reference visuals."""

        # Grid as a simple ground reference.
        self._grid = self.server.scene.add_grid(
            f"{self._ref_root}/grid",
            width=float(self.cfg.grid_width),
            height=float(self.cfg.grid_height),
        )
        self._grid.visible = bool(self.cfg.show_grid)

        # Base frame at origin (z-up). Good for orientation.
        self._base_frame = self.server.scene.add_frame(
            f"{self._ref_root}/base_frame",
            axes_length=float(self.cfg.base_axes_length),
            axes_radius=float(self.cfg.base_axes_radius),
            origin_radius=float(self.cfg.base_origin_radius),
        )
        self._base_frame.visible = bool(self.cfg.show_base_frame)

    def add_scene_toggles(self) -> None:
        """Add GUI toggles for reference + debug visuals."""
        with self.server.gui.add_folder("Reference"):
            self._cb_grid = self.server.gui.add_checkbox(
                "Show grid", initial_value=bool(self.cfg.show_grid)
            )
            self._cb_base = self.server.gui.add_checkbox(
                "Show base frame", initial_value=bool(self.cfg.show_base_frame)
            )

        with self.server.gui.add_folder("Debug"):
            self._cb_link_frames = self.server.gui.add_checkbox(
                "Show link frames", initial_value=bool(self.cfg.show_link_frames)
            )

        with self.server.gui.add_folder("Robot"):
            self._cb_robot_visuals = self.server.gui.add_checkbox(
                "Show robot visuals", initial_value=bool(self.cfg.show_robot_visuals)
            )

        def _apply_visibility() -> None:
            if self._grid is not None:
                self._grid.visible = bool(self._cb_grid.value)
            if self._base_frame is not None:
                self._base_frame.visible = bool(self._cb_base.value)

            for fr in self.link_frames.values():
                fr.visible = bool(self._cb_link_frames.value)

            # Robot visuals
            for handles in self.link_visuals.values():
                for h in handles:
                    h.visible = bool(self._cb_robot_visuals.value)

        # Attach callbacks
        self._cb_grid.on_update(lambda _: _apply_visibility())
        self._cb_base.on_update(lambda _: _apply_visibility())
        self._cb_link_frames.on_update(lambda _: _apply_visibility())
        self._cb_robot_visuals.on_update(lambda _: _apply_visibility())

        # Apply initial values
        _apply_visibility()

    def _load_mesh_cached(self, path: str) -> trimesh.Trimesh:
        if path in self._mesh_cache:
            return self._mesh_cache[path]

        mesh = trimesh.load_mesh(path)
        # Manche Formate liefern Scene -> zu einem Mesh zusammenfassen
        if isinstance(mesh, trimesh.Scene):
            parts = mesh.dump()
            mesh = trimesh.util.concatenate(tuple(parts))

        if not isinstance(mesh, trimesh.Trimesh):
            raise RuntimeError(f"Unsupported mesh type from {path}: {type(mesh)}")

        self._mesh_cache[path] = mesh
        return mesh

    def create_link_frames(self, link_names: List[str]) -> None:
        """Create small coordinate frames on every link (debug).

        Parameters
        ----------
        link_names:
            Names of all links.

        Notes
        -----
        Frames are created once, and their visibility is controlled via GUI.
        """
        for name in link_names:
            fr = self.server.scene.add_frame(
                f"{self._debug_root}/links/{name}",
                show_axes=bool(self.cfg.show_link_axes),
                axes_length=0.05,
                axes_radius=0.002,
                origin_radius=0.004,
            )
            fr.visible = bool(self.cfg.show_link_frames)
            self.link_frames[name] = fr

    def create_link_visuals(self, link_visuals: Dict[str, List[dict]]) -> None:
        self.link_visuals = {}
        for link_name, visuals in link_visuals.items():
            self.link_visuals[link_name] = []
            for k, v in enumerate(visuals):
                vtype = v.get("type")
                rgba = v.get("rgba", None)

                if vtype == "box":
                    sx, sy, sz = v["size"]
                    kwargs = {}
                    if rgba is not None:
                        r, g, b, a = rgba
                        kwargs["color"] = (int(r * 255), int(g * 255), int(b * 255))
                        kwargs["opacity"] = float(a)

                    h = self.server.scene.add_box(
                        f"{self._robot_root}/visual/{link_name}/{k}",
                        dimensions=(float(sx), float(sy), float(sz)),
                        position=(0.0, 0.0, 0.0),
                        wxyz=(1.0, 0.0, 0.0, 0.0),
                        **kwargs,
                    )
                    h._origin_T = v["origin_T"]
                    h.visible = bool(self.cfg.show_robot_visuals)
                    self.link_visuals[link_name].append(h)
                    continue

                if vtype == "mesh":
                    path = v["path"]
                    scale = v.get("scale", (1.0, 1.0, 1.0))
                    mesh = self._load_mesh_cached(path)

                    # Wenn URDF rgba hat -> add_mesh_simple (feste Farbe + opacity)
                    # Sonst -> add_mesh_trimesh (nutzt ggf. GLB/Material)
                    if rgba is not None:
                        r, g, b, a = rgba
                        h = self.server.scene.add_mesh_simple(
                            f"{self._robot_root}/visual/{link_name}/{k}",
                            vertices=mesh.vertices,
                            faces=mesh.faces,
                            color=(int(r * 255), int(g * 255), int(b * 255)),
                            opacity=float(a),
                            scale=scale,
                            position=(0.0, 0.0, 0.0),
                            wxyz=(1.0, 0.0, 0.0, 0.0),
                        )
                    else:
                        h = self.server.scene.add_mesh_trimesh(
                            name=f"{self._robot_root}/visual/{link_name}/{k}",
                            mesh=mesh,
                            scale=scale,
                            position=(0.0, 0.0, 0.0),
                            wxyz=(1.0, 0.0, 0.0, 0.0),
                        )

                    h._origin_T = v["origin_T"]
                    h.visible = bool(self.cfg.show_robot_visuals)
                    self.link_visuals[link_name].append(h)
                    continue

    # ---- per-frame updates -------------------------------------------

    def update_link_poses(self, poses: Dict[str, np.ndarray]) -> None:
        """Update all scene nodes from FK poses.

        Parameters
        ----------
        poses:
            Mapping link_name -> 4x4 transform (base -> link).
        """
        # Debug frames
        for link, T_link in poses.items():
            if link in self.link_frames:
                fr = self.link_frames[link]
                fr.position = tuple(T_link[:3, 3])
                fr.wxyz = vt.SO3.from_matrix(T_link[:3, :3]).wxyz

        # Robot visuals
        for link, T_link in poses.items():
            if link not in self.link_visuals:
                continue
            for h in self.link_visuals[link]:
                T = T_link @ h._origin_T
                h.position = tuple(T[:3, 3])
                h.wxyz = vt.SO3.from_matrix(T[:3, :3]).wxyz
