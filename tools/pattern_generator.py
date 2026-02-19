"""
Parametric Line-Following Pattern Generator for IR Tracking Calibration

Generates a rounded serpentine (boustrophedon) pattern optimized for
spatial coverage on a rectangular board, suitable for line-following
robots tracked by an IR camera system.

Physical setup
--------------
- Board:   rectangular surface (default 122 x 80 cm) with black tape on white.
- Robot:   Pololu 3pi+ 32U4, line-following via 5 reflectance sensors.
- Cameras: 4 IR cameras in a 15 mm x 10 mm matrix, 1-3 m from the board.
- Tape:    19 mm or 25 mm width (25 mm recommended for reliable pickup).

Pattern layout
--------------
The serpentine consists of parallel horizontal tracks connected by U-turn
arcs.  The return path runs along the board exterior (outside the
serpentine area) so that no lines cross -- critical for unambiguous
line following.

Sync markers (short red perpendicular lines) are placed on both sides
of each U-turn, at the arc entry and exit.  When the robot crosses a
marker its firmware should emit a synchronisation packet so the IR
camera system can correlate timestamps with known path positions.

Parametric controls
-------------------
  --board-width, --board-depth    Board size (mm)
  --margin                        Keep-out from board edge (mm)
  --spacing                       Distance between parallel tracks (mm)
  --corner-radius                 U-turn radius (mm)
  --marker-offset                 (reserved, markers placed at arc boundaries)
  --tape-width                    Physical tape width (mm)
  --no-return                     Omit the exterior return path

Outputs
-------
  Matplotlib plot (default), SVG tape-layout template, ground-truth CSV.

Usage examples
--------------
    python pattern_generator.py                          # show plot
    python pattern_generator.py --svg layout.svg         # export SVG
    python pattern_generator.py --spacing 80             # denser tracks
    python pattern_generator.py --no-return --no-plot    # open serpentine only

Author: Janis
"""

import argparse
import math
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# ============================================================================
# CONFIGURATION
# ============================================================================

@dataclass
class PatternConfig:
    """All dimensions in millimeters."""

    # Board
    board_width: float = 1220.0       # mm (122 cm)
    board_depth: float = 800.0        # mm (80 cm)

    # Margins (accounts for robot half-width + safety)
    margin: float = 100.0              # mm; board edge to exterior-return centreline

    # Serpentine parameters
    track_spacing: float = 100.0      # mm (10 cm) between parallel lines
    corner_radius: float = 50.0       # mm (5 cm) -- set to spacing/2 for smooth U-turns
    arc_resolution: int = 20          # points per semicircle

    # Sync markers (red line before each U-turn)
    marker_offset: float = 60.0       # mm before the turn centre
    marker_length: float = 40.0       # mm perpendicular span of the marker

    # Tape
    tape_width: float = 25.0          # mm

    # Clearance
    return_clearance: float = 100.0   # mm centre-to-centre between return corridor and nearest arc tape

    # Corner fiducials
    corner_square: float = 100.0       # mm; black square at each board corner
    aruco_cell: float = 10.0           # mm; cell size for ArUco markers (6x6 grid = 60 mm)
    aruco_ids: tuple = (0, 1, 2, 3)    # DICT_4X4_50 IDs for TL, TR, BL, BR corners

    # Options
    exterior_return: bool = True      # close the loop via exterior (no crossing)
    closed_loop: bool = True          # connect end back to start


# ============================================================================
# ARUCO MARKER DATA (DICT_4X4_50, IDs 0-3)
# ============================================================================

# Each entry is a 6x6 grid (1-cell black border + 4x4 data).
# 1 = black, 0 = white.  Verified against OpenCV cv2.aruco.DICT_4X4_50.
ARUCO_4X4_50 = {
    0: [
        [1, 1, 1, 1, 1, 1],
        [1, 0, 1, 0, 0, 1],
        [1, 1, 0, 1, 0, 1],
        [1, 1, 1, 0, 0, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 1, 1, 1],
    ],
    1: [
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 0, 1, 1, 0, 1],
        [1, 0, 1, 0, 1, 1],
        [1, 1, 1, 1, 1, 1],
    ],
    2: [
        [1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 0, 1],
        [1, 1, 1, 0, 0, 1],
        [1, 1, 1, 0, 1, 1],
        [1, 0, 0, 1, 0, 1],
        [1, 1, 1, 1, 1, 1],
    ],
    3: [
        [1, 1, 1, 1, 1, 1],
        [1, 0, 1, 1, 0, 1],
        [1, 0, 1, 1, 0, 1],
        [1, 1, 0, 1, 1, 1],
        [1, 1, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 1],
    ],
}


# ============================================================================
# PATTERN GENERATION
# ============================================================================

def generate_serpentine(cfg: PatternConfig) -> Tuple[np.ndarray, dict]:
    """
    Generate a rounded serpentine (boustrophedon) path.

    The path consists of:
      1. Parallel horizontal tracks connected by U-turn arcs.
      2. An optional exterior return path that follows the board
         perimeter back to the start (no line crossings).
      3. Sync-marker positions placed ``marker_offset`` mm before
         each U-turn entry point.

    Returns:
        path:    Nx2 array of (x, y) coordinates in mm.
        stats:   dict with path_length, n_passes, coverage_pct, etc.
                 ``stats["markers"]`` is a list of dicts with keys
                 ``x``, ``y``, ``angle_deg`` describing each sync marker.
    """
    # Corner radius (clamped to half spacing)
    r = min(cfg.corner_radius, cfg.track_spacing / 2.0)

    # Usable area -- when the exterior return is enabled the serpentine
    # is inset so that arcs + tape don't overlap the return corridor
    # whose centreline sits at cfg.margin from each board edge.
    #   LEFT (return-path side):
    #     return_clearance is the centre-to-centre distance between the
    #     return corridor (at cfg.margin) and the leftmost arc tape
    #     centreline.  Edge-to-edge = return_clearance - tape_width.
    #   RIGHT (far side):
    #     The arc outer tape edge touches the margin line exactly
    #     (distance from margin to arc centreline = tape_width / 2).
    #   y: tape_width / 2 puts the first/last track tape edge exactly
    #      on the margin line.
    if cfg.exterior_return:
        x_min = cfg.margin + cfg.return_clearance + r
        x_max = cfg.board_width - cfg.margin - r - cfg.tape_width / 2.0
        y_min = cfg.margin + cfg.tape_width / 2.0
        y_max = cfg.board_depth - cfg.margin - cfg.tape_width / 2.0
    else:
        x_min = cfg.margin
        x_max = cfg.board_width - cfg.margin
        y_min = cfg.margin
        y_max = cfg.board_depth - cfg.margin

    usable_w = x_max - x_min
    usable_d = y_max - y_min

    # Number of horizontal passes
    n_passes = int(usable_d / cfg.track_spacing) + 1

    # Clamp to fit
    actual_span = (n_passes - 1) * cfg.track_spacing
    if actual_span > usable_d:
        n_passes -= 1
        actual_span = (n_passes - 1) * cfg.track_spacing

    # Bottom-justify: first track sits at y_min so it is as close to
    # the return corridor as geometry allows (no wasted centering gap).
    y_offset = y_min

    points: List[Tuple[float, float]] = []
    markers: List[dict] = []           # sync-marker positions
    direction = 1  # 1 = left-to-right, -1 = right-to-left

    # When using exterior return, start at the bottom-arc exit point
    # so the closed loop joins seamlessly without backtracking.
    if cfg.exterior_return:
        points.append((cfg.margin + r, y_min))

    first_track_y = y_offset  # Y of the first serpentine track

    for i in range(n_passes):
        y = y_offset + i * cfg.track_spacing

        if direction == 1:
            points.append((x_min, y))
            points.append((x_max, y))
        else:
            points.append((x_max, y))
            points.append((x_min, y))

        # U-turn arc to next pass (if not the last pass)
        if i < n_passes - 1:
            y_next = y_offset + (i + 1) * cfg.track_spacing
            y_mid = (y + y_next) / 2.0

            if direction == 1:
                cx = x_max
                # Sync markers at arc entry and exit
                markers.append({"x": x_max, "y": y, "angle_deg": 90.0})
                markers.append({"x": x_max, "y": y_next, "angle_deg": 90.0})
                arc = _semicircle_points(cx, y_mid, r,
                                         start_angle=-math.pi / 2,
                                         sweep=math.pi,
                                         clockwise=False,
                                         n=cfg.arc_resolution)
            else:
                cx = x_min
                # Sync markers at arc entry and exit
                markers.append({"x": x_min, "y": y, "angle_deg": 90.0})
                markers.append({"x": x_min, "y": y_next, "angle_deg": 90.0})
                arc = _semicircle_points(cx, y_mid, r,
                                         start_angle=-math.pi / 2,
                                         sweep=math.pi,
                                         clockwise=True,
                                         n=cfg.arc_resolution)
            points.extend(arc)

        direction *= -1

    # --- exterior return path (no crossing) ---
    if cfg.exterior_return and len(points) >= 2:
        end = points[-1]
        start = points[0]
        if abs(end[0] - start[0]) > 1.0 or abs(end[1] - start[1]) > 1.0:
            ext = cfg.margin
            ex, ey = end
            sx, sy = start
            ret_r = r  # reuse corner radius for return-path arcs
            n_quarter = max(cfg.arc_resolution // 2, 5)

            if ex > cfg.board_width / 2.0:
                corr_x = cfg.board_width - ext
            else:
                corr_x = ext

            route: List[Tuple[float, float]] = []

            if abs(corr_x - sx) < 1.0 or abs(corr_x - (sx - r)) < 1.0:
                # Same vertical corridor -- round both corners.
                going_left = (ex > corr_x)
                going_up = (ey > sy)
                sign_x = 1 if going_left else -1
                sign_y = -1 if going_up else 1

                # ---- Corner 1: horizontal-to-vertical ----
                c1x = corr_x + sign_x * ret_r
                c1y = ey + sign_y * ret_r
                route.append((c1x, ey))  # approach
                markers.append({"x": c1x, "y": ey, "angle_deg": 90.0})
                if going_left and going_up:
                    sa1, cw1 = math.pi / 2, False
                elif going_left and not going_up:
                    sa1, cw1 = -math.pi / 2, False
                elif not going_left and going_up:
                    sa1, cw1 = math.pi / 2, True
                else:
                    sa1, cw1 = -math.pi / 2, True
                arc1 = _semicircle_points(c1x, c1y, ret_r,
                                          start_angle=sa1,
                                          sweep=math.pi / 2,
                                          clockwise=cw1,
                                          n=n_quarter)
                route.extend(arc1)
                markers.append({"x": corr_x, "y": c1y, "angle_deg": 0.0})

                # ---- Corner 2: vertical-to-horizontal ----
                c2x = corr_x + sign_x * ret_r
                c2y = sy - sign_y * ret_r
                route.append((corr_x, c2y))  # corridor approach
                markers.append({"x": corr_x, "y": c2y, "angle_deg": 0.0})
                if going_up and going_left:
                    sa2, cw2 = math.pi, False
                elif not going_up and going_left:
                    sa2, cw2 = math.pi, True
                elif going_up and not going_left:
                    sa2, cw2 = 0.0, True
                else:
                    sa2, cw2 = 0.0, False
                arc2 = _semicircle_points(c2x, c2y, ret_r,
                                          start_angle=sa2,
                                          sweep=math.pi / 2,
                                          clockwise=cw2,
                                          n=n_quarter)
                route.extend(arc2)
                markers.append({"x": c2x, "y": sy, "angle_deg": 90.0})

                # Close to start (skip if arc exit matches start)
                if abs(route[-1][0] - sx) > 0.5 or abs(route[-1][1] - sy) > 0.5:
                    route.append((sx, sy))
            else:
                # Different corridor -- route via edge (sharp corners)
                route.append((corr_x, ey))
                route.append((corr_x, ext))
                route.append((sx, ext))
                if abs(route[-1][0] - sx) > 0.5 or abs(route[-1][1] - sy) > 0.5:
                    route.append((sx, sy))

            points.extend(route)

    path = np.array(points, dtype=np.float64)

    # --- Statistics ---
    diffs = np.diff(path, axis=0)
    seg_lengths = np.sqrt(diffs[:, 0] ** 2 + diffs[:, 1] ** 2)
    total_length = np.sum(seg_lengths)

    # Direction histogram (in 30-degree bins)
    angles = np.arctan2(diffs[:, 1], diffs[:, 0]) * 180.0 / math.pi
    angles = angles % 360.0

    # Approximate coverage: resample path densely, count unique grid cells
    cell_size = cfg.tape_width
    dense = interpolate_path(path, step_mm=cell_size / 2.0)
    grid_x = ((dense[:, 0] - x_min) / cell_size).astype(int)
    grid_y = ((dense[:, 1] - y_min) / cell_size).astype(int)
    unique_cells = len(set(zip(grid_x.tolist(), grid_y.tolist())))
    total_cells = int((usable_w / cell_size) * (usable_d / cell_size))
    coverage_pct = 100.0 * unique_cells / max(total_cells, 1)

    stats = {
        "n_passes": n_passes,
        "total_length_mm": round(total_length, 1),
        "total_length_m": round(total_length / 1000.0, 2),
        "usable_area_mm": (round(usable_w, 1), round(usable_d, 1)),
        "coverage_pct": round(coverage_pct, 1),
        "unique_cells": unique_cells,
        "total_cells": total_cells,
        "n_points": len(path),
        "direction_diversity_deg": _direction_diversity(angles),
        "has_exterior_return": cfg.exterior_return,
        "markers": markers,
        "n_markers": len(markers),
        "corner_radius_mm": round(r, 1),
        "first_track_y": first_track_y,
        "start_to_first_marker_mm": round(markers[0]["x"] - path[0, 0], 1) if markers else 0.0,
    }

    return path, stats


def _semicircle_points(cx: float, cy: float, r: float,
                       start_angle: float, sweep: float,
                       clockwise: bool, n: int) -> List[Tuple[float, float]]:
    """Generate arc points for a U-turn."""
    pts = []
    for i in range(n + 1):
        t = i / n
        angle = start_angle + ((-1 if clockwise else 1) * sweep * t)
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        pts.append((x, y))
    return pts


def _direction_diversity(angles: np.ndarray) -> dict:
    """Compute direction histogram in 30-degree bins."""
    bins = np.arange(0, 390, 30)
    hist, _ = np.histogram(angles, bins=bins)
    labels = [f"{int(b)}-{int(b + 30)}" for b in bins[:-1]]
    occupied = int(np.sum(hist > 0))
    return {
        "bins": dict(zip(labels, hist.tolist())),
        "occupied_bins": occupied,
        "total_bins": len(labels),
    }


# ============================================================================
# GROUND TRUTH INTERPOLATION
# ============================================================================

def interpolate_path(path: np.ndarray, step_mm: float = 1.0) -> np.ndarray:
    """
    Resample path at uniform distance intervals.

    Returns Nx3 array: (x, y, cumulative_distance_mm).
    Useful for ground-truth position lookup given odometry distance.
    """
    diffs = np.diff(path, axis=0)
    seg_lengths = np.sqrt(diffs[:, 0] ** 2 + diffs[:, 1] ** 2)
    cum_dist = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total = cum_dist[-1]

    n_samples = int(total / step_mm) + 1
    uniform_dist = np.linspace(0, total, n_samples)

    x_interp = np.interp(uniform_dist, cum_dist, path[:, 0])
    y_interp = np.interp(uniform_dist, cum_dist, path[:, 1])

    return np.column_stack([x_interp, y_interp, uniform_dist])


# ============================================================================
# SVG EXPORT
# ============================================================================

def export_svg(path: np.ndarray, cfg: PatternConfig, filename: str,
               stats: Optional[dict] = None):
    """
    Export the pattern as an SVG file (1:1 scale in mm).

    Includes:
      - Black polyline for the tape path.
      - Red perpendicular sync markers before each U-turn.
      - Blue dimension annotations for track spacing and turn radius.
      - Green/red start/end markers.
    """
    w = cfg.board_width
    h = cfg.board_depth
    tape_w = cfg.tape_width
    markers = stats["markers"] if stats else []
    r = stats["corner_radius_mm"] if stats else cfg.corner_radius

    svg_lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}">',
        # Definitions for arrowheads (dimension lines)
        '  <defs>',
        '    <marker id="arr" markerWidth="8" markerHeight="6" '
        'refX="8" refY="3" orient="auto">',
        '      <path d="M0,0 L8,3 L0,6" fill="#2266cc"/>',
        '    </marker>',
        '    <marker id="arr-rev" markerWidth="8" markerHeight="6" '
        'refX="0" refY="3" orient="auto">',
        '      <path d="M8,0 L0,3 L8,6" fill="#2266cc"/>',
        '    </marker>',
        '    <marker id="arr-green" markerWidth="8" markerHeight="6" '
        'refX="8" refY="3" orient="auto">',
        '      <path d="M0,0 L8,3 L0,6" fill="#22aa44"/>',
        '    </marker>',
        '    <marker id="arr-green-rev" markerWidth="8" markerHeight="6" '
        'refX="0" refY="3" orient="auto">',
        '      <path d="M8,0 L0,3 L8,6" fill="#22aa44"/>',
        '    </marker>',
        '  </defs>',
        # Board outline
        f'  <rect x="0" y="0" width="{w}" height="{h}" '
        f'fill="white" stroke="black" stroke-width="2"/>',
        # Usable-area dashed rectangle
        f'  <rect x="{cfg.margin}" y="{cfg.margin}" '
        f'width="{w - 2 * cfg.margin}" height="{h - 2 * cfg.margin}" '
        f'fill="none" stroke="#ccc" stroke-width="0.5" stroke-dasharray="5,5"/>',
    ]

    # --- Corner fiducials (black squares + ArUco markers) ---
    cs = cfg.corner_square
    cell = cfg.aruco_cell
    aruco_size = 6 * cell  # 60 mm for 6x6 grid
    pad = (cs - aruco_size) / 2.0  # centring offset
    # (black_x, black_y, aruco_origin_x, aruco_origin_y, aruco_id)
    corners_fid = [
        (0, 0, pad, pad, cfg.aruco_ids[0]),                                  # TL
        (w - cs, 0, w - cs + pad, pad, cfg.aruco_ids[1]),                    # TR
        (0, h - cs, pad, h - cs + pad, cfg.aruco_ids[2]),                    # BL
        (w - cs, h - cs, w - cs + pad, h - cs + pad, cfg.aruco_ids[3]),      # BR
    ]
    for bx, by, ax_orig, ay_orig, aid in corners_fid:
        # Black square
        svg_lines.append(
            f'  <rect x="{bx}" y="{by}" width="{cs}" height="{cs}" '
            f'fill="black"/>'
        )
        # White background for ArUco readability
        svg_lines.append(
            f'  <rect x="{ax_orig}" y="{ay_orig}" '
            f'width="{aruco_size}" height="{aruco_size}" fill="white"/>'
        )
        # ArUco cells
        grid = ARUCO_4X4_50.get(aid, ARUCO_4X4_50[0])
        for row_i, row in enumerate(grid):
            for col_i, val in enumerate(row):
                if val == 1:  # black cell
                    cx = ax_orig + col_i * cell
                    cy = ay_orig + row_i * cell
                    svg_lines.append(
                        f'  <rect x="{cx:.1f}" y="{cy:.1f}" '
                        f'width="{cell}" height="{cell}" fill="black"/>'
                    )

    # --- Main path as polyline ---
    pts_str = " ".join(f"{p[0]:.1f},{p[1]:.1f}" for p in path)
    svg_lines.append(
        f'  <polyline points="{pts_str}" '
        f'fill="none" stroke="black" stroke-width="{tape_w}" '
        f'stroke-linecap="round" stroke-linejoin="round"/>'
    )

    # --- Sync markers (red perpendicular lines) ---
    half = cfg.marker_length / 2.0
    for m in markers:
        mx, my, ang = m["x"], m["y"], m["angle_deg"]
        rad = math.radians(ang)
        dx = half * math.cos(rad)
        dy = half * math.sin(rad)
        svg_lines.append(
            f'  <line x1="{mx - dx:.1f}" y1="{my - dy:.1f}" '
            f'x2="{mx + dx:.1f}" y2="{my + dy:.1f}" '
            f'stroke="red" stroke-width="{tape_w}" '
            f'stroke-linecap="round"/>'
        )

    # --- Spacing indicator (between first two tracks, left side) ---
    y_first = stats["first_track_y"] if stats else path[0, 1]
    if len(markers) >= 1:
        dim_x = max(path[0, 0] - 30.0, 10.0)
        y1 = y_first
        y2 = y_first + cfg.track_spacing
        tick = 10
        # Vertical line with arrowheads
        svg_lines.append(
            f'  <line x1="{dim_x:.1f}" y1="{y1:.1f}" '
            f'x2="{dim_x:.1f}" y2="{y2:.1f}" '
            f'stroke="#2266cc" stroke-width="1.5" '
            f'marker-start="url(#arr-rev)" marker-end="url(#arr)"/>'
        )
        # Horizontal tick marks
        svg_lines.append(
            f'  <line x1="{dim_x - tick:.1f}" y1="{y1:.1f}" '
            f'x2="{dim_x + tick:.1f}" y2="{y1:.1f}" '
            f'stroke="#2266cc" stroke-width="1"/>'
        )
        svg_lines.append(
            f'  <line x1="{dim_x - tick:.1f}" y1="{y2:.1f}" '
            f'x2="{dim_x + tick:.1f}" y2="{y2:.1f}" '
            f'stroke="#2266cc" stroke-width="1"/>'
        )
        # Label
        svg_lines.append(
            f'  <text x="{dim_x - 14:.1f}" y="{(y1 + y2) / 2:.1f}" '
            f'text-anchor="middle" font-size="18" fill="#2266cc" '
            f'transform="rotate(-90, {dim_x - 14:.1f}, {(y1 + y2) / 2:.1f})">'
            f'{cfg.track_spacing:.0f} mm</text>'
        )

    # --- Radius indicator (at first U-turn) ---
    if len(markers) >= 1:
        turn_x = markers[0]["x"]  # track endpoint at first turn
        arc_cy = y_first + cfg.track_spacing / 2.0
        # Radius line from centre to arc edge
        svg_lines.append(
            f'  <line x1="{turn_x:.1f}" y1="{arc_cy:.1f}" '
            f'x2="{turn_x + r:.1f}" y2="{arc_cy:.1f}" '
            f'stroke="#2266cc" stroke-width="1.5" '
            f'marker-end="url(#arr)"/>'
        )
        # Small circle at arc centre
        svg_lines.append(
            f'  <circle cx="{turn_x:.1f}" cy="{arc_cy:.1f}" '
            f'r="3" fill="#2266cc"/>'
        )
        # Label
        svg_lines.append(
            f'  <text x="{turn_x + r / 2:.1f}" y="{arc_cy - 8:.1f}" '
            f'text-anchor="middle" font-size="18" fill="#2266cc">'
            f'r={r:.0f} mm</text>'
        )

    # --- Start / end markers ---
    svg_lines.append(
        f'  <circle cx="{path[0, 0]:.1f}" cy="{path[0, 1]:.1f}" '
        f'r="10" fill="green" opacity="0.7"/>'
    )
    svg_lines.append(
        f'  <circle cx="{path[-1, 0]:.1f}" cy="{path[-1, 1]:.1f}" '
        f'r="10" fill="red" opacity="0.7"/>'
    )

    # --- Start-to-first-marker dimension (horizontal, above first track) ---
    s2m_dist = stats.get("start_to_first_marker_mm", 0) if stats else 0
    if s2m_dist > 0 and markers:
        s2m_track_y = stats.get("first_track_y", path[0, 1]) if stats else path[0, 1]
        dim_y = s2m_track_y - cfg.tape_width - 15
        start_x = path[0, 0]
        marker_x = markers[0]["x"]
        svg_lines.append(
            f'  <line x1="{start_x:.1f}" y1="{dim_y:.1f}" '
            f'x2="{marker_x:.1f}" y2="{dim_y:.1f}" '
            f'stroke="#22aa44" stroke-width="1.5" '
            f'marker-start="url(#arr-green-rev)" marker-end="url(#arr-green)"/>'
        )
        svg_lines.append(
            f'  <line x1="{start_x:.1f}" y1="{dim_y - 8:.1f}" '
            f'x2="{start_x:.1f}" y2="{dim_y + 8:.1f}" '
            f'stroke="#22aa44" stroke-width="1"/>'
        )
        svg_lines.append(
            f'  <line x1="{marker_x:.1f}" y1="{dim_y - 8:.1f}" '
            f'x2="{marker_x:.1f}" y2="{dim_y + 8:.1f}" '
            f'stroke="#22aa44" stroke-width="1"/>'
        )
        mid_x = (start_x + marker_x) / 2.0
        svg_lines.append(
            f'  <text x="{mid_x:.1f}" y="{dim_y - 10:.1f}" '
            f'text-anchor="middle" font-size="16" fill="#22aa44">'
            f'start to 1st marker: {s2m_dist:.0f} mm</text>'
        )

    # --- Margin indicator (bottom edge, from board edge to margin line) ---
    margin_dim_y = h - cfg.margin / 2.0  # place below the margin line
    margin_dim_x1 = w / 2.0 - 60.0      # centred-ish span
    svg_lines.append(
        f'  <line x1="{margin_dim_x1:.1f}" y1="{h:.1f}" '
        f'x2="{margin_dim_x1:.1f}" y2="{h - cfg.margin:.1f}" '
        f'stroke="#cc6622" stroke-width="1.5" '
        f'marker-start="url(#arr-rev)" marker-end="url(#arr)"/>'
    )
    # Tick marks
    svg_lines.append(
        f'  <line x1="{margin_dim_x1 - 10:.1f}" y1="{h:.1f}" '
        f'x2="{margin_dim_x1 + 10:.1f}" y2="{h:.1f}" '
        f'stroke="#cc6622" stroke-width="1"/>'
    )
    svg_lines.append(
        f'  <line x1="{margin_dim_x1 - 10:.1f}" y1="{h - cfg.margin:.1f}" '
        f'x2="{margin_dim_x1 + 10:.1f}" y2="{h - cfg.margin:.1f}" '
        f'stroke="#cc6622" stroke-width="1"/>'
    )
    svg_lines.append(
        f'  <text x="{margin_dim_x1 + 16:.1f}" y="{h - cfg.margin / 2.0:.1f}" '
        f'text-anchor="start" font-size="16" fill="#cc6622">'
        f'margin {cfg.margin:.0f} mm</text>'
    )

    # --- Board dimension labels ---
    svg_lines.append(
        f'  <text x="{w / 2}" y="{h - 10}" text-anchor="middle" '
        f'font-size="30" fill="#666">{w / 10:.0f} cm</text>'
    )
    svg_lines.append(
        f'  <text x="15" y="{h / 2}" text-anchor="middle" '
        f'font-size="30" fill="#666" '
        f'transform="rotate(-90, 15, {h / 2})">{h / 10:.0f} cm</text>'
    )

    svg_lines.append('</svg>')

    with open(filename, 'w') as f:
        f.write('\n'.join(svg_lines))


# ============================================================================
# CSV EXPORT (ground truth)
# ============================================================================

def export_ground_truth_csv(path: np.ndarray, filename: str,
                            step_mm: float = 5.0):
    """
    Export uniformly-sampled ground truth positions to CSV.

    Columns: distance_mm, x_mm, y_mm
    Can be compared against IR-camera-tracked positions.
    """
    resampled = interpolate_path(path, step_mm)
    with open(filename, 'w') as f:
        f.write("distance_mm,x_mm,y_mm\n")
        for row in resampled:
            f.write(f"{row[2]:.1f},{row[0]:.1f},{row[1]:.1f}\n")


# ============================================================================
# VISUALIZATION
# ============================================================================

def plot_pattern(path: np.ndarray, cfg: PatternConfig, stats: dict):
    """Display the pattern with matplotlib, including sync markers and
    dimension annotations for track spacing and turn radius."""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("[WARN] matplotlib not installed; skipping plot")
        return

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))

    # Board outline
    board = mpatches.Rectangle((0, 0), cfg.board_width, cfg.board_depth,
                                linewidth=2, edgecolor='black',
                                facecolor='#f5f5f5')
    ax.add_patch(board)

    # Usable area
    margin_rect = mpatches.Rectangle(
        (cfg.margin, cfg.margin),
        cfg.board_width - 2 * cfg.margin,
        cfg.board_depth - 2 * cfg.margin,
        linewidth=1, edgecolor='gray', facecolor='none',
        linestyle='--', label='Usable area'
    )
    ax.add_patch(margin_rect)

    # --- Corner fiducials (black squares + ArUco markers) ---
    cs = cfg.corner_square
    cell = cfg.aruco_cell
    aruco_size = 6 * cell
    pad = (cs - aruco_size) / 2.0
    corners_fid = [
        (0, 0, pad, pad, cfg.aruco_ids[0]),
        (cfg.board_width - cs, 0, cfg.board_width - cs + pad, pad, cfg.aruco_ids[1]),
        (0, cfg.board_depth - cs, pad, cfg.board_depth - cs + pad, cfg.aruco_ids[2]),
        (cfg.board_width - cs, cfg.board_depth - cs,
         cfg.board_width - cs + pad, cfg.board_depth - cs + pad, cfg.aruco_ids[3]),
    ]
    for bx, by, ax_orig, ay_orig, aid in corners_fid:
        ax.add_patch(mpatches.Rectangle(
            (bx, by), cs, cs, facecolor='black', edgecolor='none'))
        # White background
        ax.add_patch(mpatches.Rectangle(
            (ax_orig, ay_orig), aruco_size, aruco_size,
            facecolor='white', edgecolor='none'))
        # ArUco cells
        grid = ARUCO_4X4_50.get(aid, ARUCO_4X4_50[0])
        for row_i, row in enumerate(grid):
            for col_i, val in enumerate(row):
                if val == 1:
                    ax.add_patch(mpatches.Rectangle(
                        (ax_orig + col_i * cell,
                         ay_orig + row_i * cell),
                        cell, cell,
                        facecolor='black', edgecolor='none'))

    # Path with tape width
    ax.plot(path[:, 0], path[:, 1], 'k-', linewidth=cfg.tape_width / 10,
            solid_capstyle='round', solid_joinstyle='round', label='Tape path')

    # Centerline
    ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=0.5, alpha=0.5,
            label='Centerline')

    # --- Sync markers (red perpendicular lines) ---
    half = cfg.marker_length / 2.0
    for m in stats.get("markers", []):
        mx, my, ang = m["x"], m["y"], m["angle_deg"]
        rad = math.radians(ang)
        dx_m = half * math.cos(rad)
        dy_m = half * math.sin(rad)
        ax.plot([mx - dx_m, mx + dx_m], [my - dy_m, my + dy_m],
                'r-', linewidth=2.5, solid_capstyle='round')
    if stats.get("markers"):
        ax.plot([], [], 'r-', linewidth=2.5, label='Sync marker')

    # Start / end markers
    ax.plot(path[0, 0], path[0, 1], 'go', markersize=12, label='Start')
    ax.plot(path[-1, 0], path[-1, 1], 'rs', markersize=12, label='End')

    # Direction arrows along path
    n_arrows = 8
    indices = np.linspace(0, len(path) - 2, n_arrows, dtype=int)
    for idx in indices:
        ddx = path[idx + 1, 0] - path[idx, 0]
        ddy = path[idx + 1, 1] - path[idx, 1]
        norm = math.sqrt(ddx * ddx + ddy * ddy)
        if norm > 0:
            ax.annotate('', xy=(path[idx, 0] + ddx, path[idx, 1] + ddy),
                        xytext=(path[idx, 0], path[idx, 1]),
                        arrowprops=dict(arrowstyle='->', color='orange',
                                        lw=1.5))

    # --- Spacing indicator (blue dimension line between first two tracks) ---
    y0 = stats.get("first_track_y", path[0, 1])
    y1_dim = y0 + cfg.track_spacing
    dim_x = max(path[0, 0] - 30.0, 10.0)
    ax.annotate('', xy=(dim_x, y1_dim), xytext=(dim_x, y0),
                arrowprops=dict(arrowstyle='<->', color='#2266cc', lw=1.5))
    ax.text(dim_x - 12, (y0 + y1_dim) / 2.0,
            f'{cfg.track_spacing:.0f} mm',
            ha='center', va='center', fontsize=8, color='#2266cc',
            rotation=90, fontweight='bold',
            bbox=dict(facecolor='white', edgecolor='none', pad=1))

    # --- Margin indicator (orange dimension, bottom edge) ---
    margin_dim_x = cfg.board_width / 2.0 - 60.0
    ax.annotate('', xy=(margin_dim_x, cfg.board_depth),
                xytext=(margin_dim_x, cfg.board_depth - cfg.margin),
                arrowprops=dict(arrowstyle='<->', color='#cc6622', lw=1.5))
    ax.text(margin_dim_x + 14, cfg.board_depth - cfg.margin / 2.0,
            f'margin {cfg.margin:.0f} mm',
            ha='left', va='center', fontsize=8, color='#cc6622',
            fontweight='bold',
            bbox=dict(facecolor='white', edgecolor='none', pad=1))

    # --- Radius indicator (blue, at first right-side U-turn) ---
    r = stats.get("corner_radius_mm", cfg.corner_radius)
    turn_x = stats["markers"][0]["x"] if stats.get("markers") else (cfg.board_width - cfg.margin)
    arc_cy = y0 + cfg.track_spacing / 2.0
    ax.annotate('', xy=(turn_x + r, arc_cy), xytext=(turn_x, arc_cy),
                arrowprops=dict(arrowstyle='->', color='#2266cc', lw=1.5))
    ax.plot(turn_x, arc_cy, 'o', color='#2266cc', markersize=4)
    ax.text(turn_x + r / 2.0, arc_cy - 12,
            f'r = {r:.0f} mm',
            ha='center', va='center', fontsize=8, color='#2266cc',
            fontweight='bold',
            bbox=dict(facecolor='white', edgecolor='none', pad=1))

    # --- Start-to-first-marker dimension (green, horizontal) ---
    s2m = stats.get("start_to_first_marker_mm", 0)
    if s2m > 0 and stats.get("markers"):
        s2m_y = y0 - cfg.tape_width - 15
        start_x = path[0, 0]
        marker_x = stats["markers"][0]["x"]
        ax.annotate('', xy=(marker_x, s2m_y), xytext=(start_x, s2m_y),
                    arrowprops=dict(arrowstyle='<->', color='#22aa44', lw=1.5))
        ax.text((start_x + marker_x) / 2.0, s2m_y - 8,
                f'start to 1st marker: {s2m:.0f} mm',
                ha='center', va='center', fontsize=8, color='#22aa44',
                fontweight='bold',
                bbox=dict(facecolor='white', edgecolor='none', pad=1))

    ax.set_xlim(-20, cfg.board_width + 20)
    ax.set_ylim(-20, cfg.board_depth + 20)
    ax.set_aspect('equal')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title(
        f'Serpentine Pattern  |  {stats["n_passes"]} passes  |  '
        f'{stats["total_length_m"]} m  |  '
        f'{stats["coverage_pct"]:.0f}% coverage  |  '
        f'{stats["n_markers"]} sync markers'
        f'{"  |  + exterior return" if stats["has_exterior_return"] else ""}'
    )
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Stats text box
    info = (
        f'Board: {cfg.board_width / 10:.0f} x {cfg.board_depth / 10:.0f} cm\n'
        f'Margin: {cfg.margin:.0f} mm\n'
        f'Spacing: {cfg.track_spacing / 10:.0f} cm\n'
        f'Corner r: {r:.0f} mm\n'
        f'Tape: {cfg.tape_width:.0f} mm\n'
        f'Path: {stats["total_length_m"]} m\n'
        f'Passes: {stats["n_passes"]}\n'
        f'Markers: {stats["n_markers"]}\n'
        f'Direction bins: '
        f'{stats["direction_diversity_deg"]["occupied_bins"]}/'
        f'{stats["direction_diversity_deg"]["total_bins"]}'
    )
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax.text(0.01, 0.99, info, transform=ax.transAxes, fontsize=8,
            verticalalignment='top', bbox=props, family='monospace')

    plt.tight_layout()
    plt.show()


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Generate parametric line-following pattern for IR tracking calibration')

    # Board
    parser.add_argument('--board-width', type=float, default=1220.0,
                        help='Board width in mm (default: 1220)')
    parser.add_argument('--board-depth', type=float, default=800.0,
                        help='Board depth in mm (default: 800)')
    parser.add_argument('--margin', type=float, default=100.0,
                        help='Margin from board edge in mm (default: 100)')

    # Pattern
    parser.add_argument('--spacing', type=float, default=100.0,
                        help='Track spacing in mm (default: 100)')
    parser.add_argument('--corner-radius', type=float, default=50.0,
                        help='U-turn corner radius in mm (default: 50)')
    parser.add_argument('--marker-offset', type=float, default=60.0,
                        help='Sync marker distance before turn in mm (default: 60)')
    parser.add_argument('--no-return', action='store_true',
                        help='Omit exterior return path')
    parser.add_argument('--return-clearance', type=float, default=100.0,
                        help='Centre-to-centre distance between return corridor and nearest arc (default: 100)')

    # Tape
    parser.add_argument('--tape-width', type=float, default=25.0,
                        help='Tape width in mm (default: 25)')

    # Output
    parser.add_argument('--svg', type=str, default=None,
                        help='Export SVG file path')
    parser.add_argument('--csv', type=str, default=None,
                        help='Export ground-truth CSV file path')
    parser.add_argument('--csv-interval', type=float, default=5.0,
                        help='CSV ground-truth sample interval in mm (default: 5)')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip matplotlib plot')

    args = parser.parse_args()

    cfg = PatternConfig(
        board_width=args.board_width,
        board_depth=args.board_depth,
        margin=args.margin,
        track_spacing=args.spacing,
        corner_radius=args.corner_radius,
        marker_offset=args.marker_offset,
        tape_width=args.tape_width,
        return_clearance=args.return_clearance,
        exterior_return=not args.no_return,
    )

    # Generate
    path, stats = generate_serpentine(cfg)

    # Print stats
    print("=" * 60)
    print("PATTERN STATISTICS")
    print("=" * 60)
    print(f"  Board:           {cfg.board_width / 10:.0f} x {cfg.board_depth / 10:.0f} cm")
    print(f"  Usable area:     {stats['usable_area_mm'][0] / 10:.0f} x "
          f"{stats['usable_area_mm'][1] / 10:.0f} cm")
    print(f"  Margin:          {cfg.margin:.0f} mm")
    print(f"  Track spacing:   {cfg.track_spacing / 10:.0f} cm")
    print(f"  Corner radius:   {stats['corner_radius_mm']} mm")
    print(f"  Tape width:      {cfg.tape_width:.0f} mm")
    print(f"  Passes:          {stats['n_passes']}")
    print(f"  Path length:     {stats['total_length_m']} m "
          f"({stats['total_length_mm']:.0f} mm)")
    print(f"  Coverage:        {stats['coverage_pct']:.1f}%")
    print(f"  Exterior return: {'yes' if stats['has_exterior_return'] else 'no'}")
    print(f"  Sync markers:    {stats['n_markers']}")
    print(f"  Return clearance:{cfg.return_clearance:.0f} mm")
    if stats.get('start_to_first_marker_mm'):
        print(f"  Start to 1st mk: {stats['start_to_first_marker_mm']:.0f} mm")
    div = stats['direction_diversity_deg']
    print(f"  Direction bins:  {div['occupied_bins']}/{div['total_bins']} "
          f"(30-degree bins with samples)")
    print(f"  Path points:     {stats['n_points']}")
    print("=" * 60)

    # Exports
    if args.svg:
        export_svg(path, cfg, args.svg, stats)
        print(f"  SVG exported:    {args.svg}")

    if args.csv:
        export_ground_truth_csv(path, args.csv, args.csv_interval)
        print(f"  CSV exported:    {args.csv}")

    # Plot
    if not args.no_plot:
        plot_pattern(path, cfg, stats)


if __name__ == '__main__':
    main()
