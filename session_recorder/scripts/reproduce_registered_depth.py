#!/usr/bin/env python3
"""Reproduce colour-registered depth from raw sd depth + a calibration snapshot.

This is the offline counterpart to recording raw sd depth instead of the
bridge's qhd `image_depth_rect`: it proves the registered/rectified product
can be regenerated from what we now store (raw sd depth + camera_info + the
kinect2_bridge calib_*.yaml snapshot copied into each session).

Pipeline (mirrors kinect2_registration):
  1. undistort the raw depth using the IR/depth intrinsics (calib_ir.yaml)
  2. back-project each depth pixel to a 3D point in the depth frame
  3. transform into the colour frame with the depth->colour extrinsic
     (calib_pose.yaml: rotation + translation)
  4. project into the colour image with the colour intrinsics (calib_color.yaml)
  5. z-buffer the nearest depth per colour pixel -> registered depth image

Usage:
  # regenerate registered depth for a single 16-bit depth PNG
  reproduce_registered_depth.py --calib <session>/calibration/<serial> \\
      --depth raw_depth.png --out registered_depth.png

  # numeric self-test (no data needed): synthesises a scene, registers it,
  # and checks the geometry round-trips
  reproduce_registered_depth.py --selftest

Depends only on numpy + opencv (cv2), so it runs anywhere, not just on ROS.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - guidance only
    cv2 = None


COLOR_SIZE = (1920, 1080)  # kinect v2 colour (w, h); qhd is this / 2


def _read_opencv_yaml(path: str, key: str) -> np.ndarray:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"cannot open {path}")
    node = fs.getNode(key)
    if node.empty():
        raise KeyError(f"'{key}' not found in {path}")
    mat = node.mat()
    fs.release()
    return np.asarray(mat, dtype=np.float64)


class Calibration:
    """Intrinsics + depth->colour extrinsic loaded from a bridge snapshot."""

    def __init__(self, ir_K, ir_D, color_K, color_D, R, t):
        self.ir_K = ir_K
        self.ir_D = ir_D
        self.color_K = color_K
        self.color_D = color_D
        self.R = R
        self.t = t.reshape(3)

    @classmethod
    def from_dir(cls, calib_dir: str) -> "Calibration":
        ir = os.path.join(calib_dir, "calib_ir.yaml")
        color = os.path.join(calib_dir, "calib_color.yaml")
        pose = os.path.join(calib_dir, "calib_pose.yaml")
        return cls(
            ir_K=_read_opencv_yaml(ir, "cameraMatrix"),
            ir_D=_read_opencv_yaml(ir, "distortionCoefficients").reshape(-1),
            color_K=_read_opencv_yaml(color, "cameraMatrix"),
            color_D=_read_opencv_yaml(color, "distortionCoefficients").reshape(-1),
            R=_read_opencv_yaml(pose, "rotation"),
            t=_read_opencv_yaml(pose, "translation"),
        )


def register_depth(
    depth: np.ndarray,
    calib: Calibration,
    color_size: tuple[int, int] = COLOR_SIZE,
    color_scale: float = 1.0,
) -> np.ndarray:
    """Register a raw sd depth image (uint16 mm) into the colour frame.

    color_scale downsizes the colour intrinsics/output (e.g. 0.5 for qhd).
    Returns a uint16 depth image (mm) at the (scaled) colour resolution.
    """
    h, w = depth.shape
    color_w = int(round(color_size[0] * color_scale))
    color_h = int(round(color_size[1] * color_scale))
    color_K = calib.color_K.copy()
    color_K[:2, :] *= color_scale

    # 1. undistort depth on its own grid. INTER_NEAREST: never interpolate
    # across depth discontinuities (that invents flying pixels).
    map1, map2 = cv2.initUndistortRectifyMap(
        calib.ir_K, calib.ir_D, None, calib.ir_K, (w, h), cv2.CV_32FC1
    )
    depth_u = cv2.remap(depth, map1, map2, cv2.INTER_NEAREST)

    # 2. back-project valid pixels to 3D (metres) in the depth frame.
    ys, xs = np.nonzero(depth_u)
    z = depth_u[ys, xs].astype(np.float64) / 1000.0
    fx, fy = calib.ir_K[0, 0], calib.ir_K[1, 1]
    cx, cy = calib.ir_K[0, 2], calib.ir_K[1, 2]
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy
    pts = np.stack([x, y, z], axis=1)  # (N,3)

    # 3. depth -> colour frame.
    pts_c = pts @ calib.R.T + calib.t

    # 4. project into the colour image.
    zc = pts_c[:, 2]
    valid = zc > 0
    pts_c, zc = pts_c[valid], zc[valid]
    u = (color_K[0, 0] * pts_c[:, 0] / zc + color_K[0, 2]).round().astype(int)
    v = (color_K[1, 1] * pts_c[:, 1] / zc + color_K[1, 2]).round().astype(int)
    inside = (u >= 0) & (u < color_w) & (v >= 0) & (v < color_h)
    u, v, zc = u[inside], v[inside], zc[inside]

    # 5. nearest-depth z-buffer.
    registered = np.full((color_h, color_w), np.inf, dtype=np.float64)
    flat = v * color_w + u
    order = np.argsort(-zc)  # write far first so near overwrites
    np.minimum.at(registered.reshape(-1), flat[order], zc[order] * 1000.0)
    registered[np.isinf(registered)] = 0
    return registered.astype(np.uint16)


def _selftest() -> int:
    """Synthesise a fronto-parallel wall, register it, verify the geometry."""
    if cv2 is None:
        print("SELFTEST SKIPPED: opencv (cv2) not installed")
        return 0
    rng = np.random.default_rng(0)
    w, h = 512, 424
    ir_K = np.array([[366.0, 0, 256.0], [0, 366.0, 212.0], [0, 0, 1]])
    ir_D = np.zeros(5)
    color_K = np.array([[1081.0, 0, 959.5], [0, 1081.0, 539.5], [0, 0, 1]])
    # 5.2 cm baseline, essentially identity rotation (real Kinect values).
    R = np.eye(3)
    t = np.array([-0.052, 0.0, 0.0])
    calib = Calibration(ir_K, ir_D, color_K, color_K, R, t)

    depth = np.full((h, w), 2000, dtype=np.uint16)  # flat wall at 2 m
    depth[rng.random((h, w)) < 0.02] = 0  # a few invalid pixels

    reg = register_depth(depth, calib, color_scale=0.5)  # qhd output

    assert reg.shape == (540, 960), reg.shape
    filled = reg[reg > 0]
    assert filled.size > 0, "no depth registered"
    # A 2 m wall stays ~2 m after a 5 cm lateral shift.
    med = np.median(filled)
    assert 1900 <= med <= 2100, f"median depth {med} mm off"
    # Registered points should occupy a plausible fraction of the colour FOV.
    coverage = filled.size / reg.size
    assert coverage > 0.2, f"coverage {coverage:.2f} too low"
    print(
        f"SELFTEST PASS: registered {filled.size} px, median {med:.0f} mm, "
        f"coverage {coverage:.0%}, output {reg.shape}"
    )
    return 0


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--calib", help="calibration snapshot dir for the camera")
    ap.add_argument("--depth", help="raw sd depth image (16-bit PNG, mm)")
    ap.add_argument("--out", help="output registered depth PNG")
    ap.add_argument("--qhd", action="store_true", help="output at qhd (0.5x colour)")
    ap.add_argument("--selftest", action="store_true", help="run numeric self-test")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()

    if cv2 is None:
        print("ERROR: opencv (cv2) is required for real data", file=sys.stderr)
        return 2
    if not (args.calib and args.depth and args.out):
        ap.error("--calib, --depth and --out are required (or use --selftest)")

    calib = Calibration.from_dir(args.calib)
    depth = cv2.imread(args.depth, cv2.IMREAD_UNCHANGED)
    if depth is None:
        print(f"ERROR: could not read {args.depth}", file=sys.stderr)
        return 2
    reg = register_depth(depth, calib, color_scale=0.5 if args.qhd else 1.0)
    cv2.imwrite(args.out, reg)
    filled = int((reg > 0).sum())
    print(f"wrote {args.out}  ({reg.shape[1]}x{reg.shape[0]}, {filled} px with depth)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
