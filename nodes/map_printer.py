#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path

# NEW: force a headless backend BEFORE importing pyplot
import matplotlib as _mpl
_mpl.use("Agg")

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.tri as mtri
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
import csv

@dataclass
class PathFeatures:
    x: np.ndarray
    y: np.ndarray
    height: np.ndarray
    density: Optional[np.ndarray]  # can be None

class PathMapPrinter:
    """
    One-file, no-ROS helper you can call from GUI.
    - Loads CSV with headers: x, y, height[, density]
    - Makes 3 images: path, gradient heatmap, density heatmap
    - Fixed frame: 48.5 x 48.5 m centered at (0,0) (clips out-of-bounds points)
    """
    def __init__(self, fixed_extent_m: float = 48.5):
        self.fixed_extent_m = float(fixed_extent_m)

    # ---- Public convenience: latest CSV in a folder ----
    def latest_csv(self, data_dir: Path) -> Optional[Path]:
        data_dir = Path(data_dir)
        if not data_dir.exists():
            return None
        csvs = sorted(data_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        return csvs[0] if csvs else None

    # ---- Public: generate directly from CSV path ----
    def generate_from_csv(self,
                          csv_path: Path,
                          out_dir: Path,
                          base_name: Optional[str] = None,
                          bg_img: Optional[Path] = None,
                          style: str = "contour",
                          levels: int = 20,
                          alpha: float = 0.75) -> Dict[str, str]:
        pf = self._load_csv(csv_path)
        if base_name is None:
            base_name = csv_path.stem
        return self.generate_maps(pf, out_dir, base_name, bg_img, style, levels, alpha)

    # ---- Core plotting ----
    def generate_maps(self,
                      pf: PathFeatures,
                      out_dir: Path,
                      base_name: str,
                      bg_img: Optional[Path] = None,
                      style: str = "contour",
                      levels: int = 20,
                      alpha: float = 0.75) -> Dict[str, str]:
        out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
        x, y, h, d = self._clip_to_extent(pf.x, pf.y, pf.height, pf.density)
        if x.size < 3:
            raise ValueError("Not enough in-bounds samples after clipping.")

        extent = self._extent_tuple()
        # 1) Path
        fig1, ax1 = plt.subplots(figsize=(7, 6), dpi=150)
        self._draw_bg(ax1, bg_img, extent)
        ax1.plot(x, y, linewidth=2)
        ax1.scatter([x[0]], [y[0]], s=50, marker="o")
        ax1.scatter([x[-1]], [y[-1]], s=50, marker="X")
        ax1.set_title("Drone Path (start ●, end ✕)")
        self._fix_axes(ax1, extent)
        p1 = out / f"{base_name}_path.png"
        fig1.tight_layout(); fig1.savefig(p1); plt.close(fig1)

        # 2) Gradient (|Δh|/Δs)
        grad = self._grad_along_path(x, y, h)
        fig2, ax2 = plt.subplots(figsize=(7, 6), dpi=150)
        self._draw_bg(ax2, bg_img, extent)
        # use larger dots for thicker “heat” line
        obj2 = self._heat(ax2, x, y, grad, style, levels, alpha, dot_size=220)
        # optional thin outline for definition
        ax2.plot(x, y, linewidth=1.2, color='k', alpha=0.35)
        ax2.set_title("Gradient Along Path (|Δh|/Δs)")
        self._fix_axes(ax2, extent)
        cb2 = fig2.colorbar(obj2 if hasattr(obj2, "collections") else obj2, ax=ax2, fraction=0.046, pad=0.04)
        cb2.set_label("Slope (m/m)")
        p2 = out / f"{base_name}_gradient.png"
        fig2.tight_layout(); fig2.savefig(p2); plt.close(fig2)

        # 3) Density
        if d is None:
            d = self._proxy_density(x, y)
        fig3, ax3 = plt.subplots(figsize=(7, 6), dpi=150)
        self._draw_bg(ax3, bg_img, extent)
        obj3 = self._heat(ax3, x, y, d, style, levels, alpha, dot_size=220)
        ax3.plot(x, y, linewidth=1.2, color='k', alpha=0.35)
        ax3.set_title("Object Density Along Path")
        self._fix_axes(ax3, extent)
        cb3 = fig3.colorbar(obj3 if hasattr(obj3, "collections") else obj3, ax=ax3, fraction=0.046, pad=0.04)
        cb3.set_label("Objects / m² (or unitless density)")
        p3 = out / f"{base_name}_density.png"
        fig3.tight_layout(); fig3.savefig(p3); plt.close(fig3)

        return {
            "path_png": str(p1),
            "gradient_png": str(p2),
            "density_png": str(p3)
        }

    # ================= helpers =================
    def _load_csv(self, path: Path) -> PathFeatures:
        xs, ys, hs, ds = [], [], [], []
        with open(path, "r", newline="") as f:
            reader = csv.DictReader(f)
            fields = {k.strip().lower() for k in (reader.fieldnames or [])}
            if not {"x", "y", "height"}.issubset(fields):
                raise ValueError(f"{path} must have columns at least: x, y, height")
            has_density = "density" in fields
            for row in reader:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
                hs.append(float(row["height"]))
                if has_density:
                    ds.append(float(row["density"]))
        x, y, h = np.asarray(xs), np.asarray(ys), np.asarray(hs)
        d = np.asarray(ds) if ds else None
        return PathFeatures(x, y, h, d)

    def _extent_tuple(self) -> Tuple[float, float, float, float]:
        half = self.fixed_extent_m / 2.0
        return (-half, half, -half, half)

    def _clip_to_extent(self, x, y, h, d=None):
        xmin, xmax, ymin, ymax = self._extent_tuple()
        m = (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax) & np.isfinite(h)
        x, y, h = x[m], y[m], h[m]
        if d is not None:
            d = d[m]
        return x, y, h, d

    def _grad_along_path(self, x, y, h):
        dx = np.diff(x, prepend=x[0]); dy = np.diff(y, prepend=y[0])
        ds = np.hypot(dx, dy); dh = np.diff(h, prepend=h[0])
        g = np.abs(dh) / np.maximum(ds, 1e-9)
        g[~np.isfinite(g)] = 0.0
        if g.size > 4:
            k = np.array([0.1, 0.2, 0.4, 0.2, 0.1])
            pad = len(k)//2
            g = np.convolve(np.pad(g,(pad,pad),mode="edge"), k, mode="valid")
        return g

    def _proxy_density(self, x, y):
        """
        If CSV has no 'density', build a smooth path-sample density proxy
        using triangulation—higher where samples cluster (like slower/denser areas).
        """
        tri = mtri.Triangulation(x, y)
        # local interpolation of sample count via triangle areas
        # smaller triangles → higher "density"
        tris = tri.triangles
        A = 0.5 * np.abs(
            (x[tris[:,1]] - x[tris[:,0]]) * (y[tris[:,2]] - y[tris[:,0]]) -
            (x[tris[:,2]] - x[tris[:,0]]) * (y[tris[:,1]] - y[tris[:,0]])
        )
        nodal = np.zeros_like(x)
        counts = np.zeros_like(x)
        for i, tri_idx in enumerate(tris):
            area = A[i]
            if area > 0:
                val = 1.0 / area
                for n in tri_idx:
                    nodal[n] += val
                    counts[n] += 1.0
        counts[counts == 0] = 1.0
        d = nodal / counts
        # normalise for display
        d = (d - d.min()) / (d.ptp() + 1e-9)
        return d

    def _draw_bg(self, ax, bg_img: Optional[Path], extent):
        if not bg_img:
            return
        import matplotlib.image as mpimg
        img = mpimg.imread(str(bg_img))
        ax.imshow(img, extent=extent, origin="upper")

    def _fix_axes(self, ax, extent):
        ax.set_aspect("equal"); ax.grid(True, alpha=0.2)
        ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3])
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")

    def _heat(self, ax, x, y, v, style: str, levels: int, alpha: float, dot_size: int = 180):
        """Draw either a smooth filled contour or a thick coloured line from points."""
        if style == "contour":
            tri = mtri.Triangulation(x, y)
            cf = ax.tricontourf(tri, v, levels=levels, alpha=alpha)
            return cf
        else:
            # dots mode – appears like a thick heat-coloured path
            sc = ax.scatter(
                x, y,
                c=v,
                s=dot_size,            # marker size (controls line thickness)
                alpha=alpha,
                edgecolors="none"
            )
            return sc

