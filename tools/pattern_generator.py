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

    # Options
    exterior_return: bool = True      # close the loop via exterior (no crossing)
    closed_loop: bool = True          # connect end back to start


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
    # is inset so that arcs + tape clear the return corridor whose
    # centreline sits at cfg.margin from each board edge.
    # The gap between return-tape edge and arc-tape edge equals one
    # tape_width (the "2 *" accounts for both paths' half-widths
    # plus one full tape-width clearance).
    if cfg.exterior_return:
        x_min = cfg.margin + 2 * cfg.tape_width + r
        x_max = cfg.board_width - cfg.margin - 2 * cfg.tape_width - r
        y_min = cfg.margin + 2 * cfg.tape_width
        y_max = cfg.board_depth - cfg.margin - 2 * cfg.tape_width
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

    # Centre vertically
    y_offset = y_min + (usable_d - actual_span) / 2.0

    points: List[Tuple[float, float]] = []
    markers: List[dict] = []           # sync-marker positions
    direction = 1  # 1 = left-to-right, -1 = right-to-left

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
                                         start_angle=math.pi / 2,
                                         sweep=math.pi,
                                         clockwise=False,
                                         n=cfg.arc_resolution)
            points.extend(arc)

        direction *= -1

    # --- exterior return path (no crossing) ---
    if cfg.exterior_return and len(points) >= 2:
        end = points[-1]
        start = points[0]
        if abs(end[0] - start[0]) > 1.0 or abs(end[1] - start[1]) > 1.0:
            # Route along the board perimeter from end back to start.
            # The exterior corridor centreline sits at cfg.margin from
            # the board edge -- well clear of the inset serpentine.
            ext = cfg.margin  # corridor centreline offset from board edge

            ex, ey = end
            sx, sy = start

            route: List[Tuple[float, float]] = []

            # Move horizontally to the nearest exterior corridor
            if ex > cfg.board_width / 2.0:
                corr_x = cfg.board_width - ext
            else:
                corr_x = ext
            route.append((corr_x, ey))

            # Travel down to the bottom corridor
            route.append((corr_x, ext))

            # Move along the bottom corridor to beneath the start
            route.append((sx, ext))

            # Rise up to the start point
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
        '  </defs>',
        # Board outline
        f'  <rect x="0" y="0" width="{w}" height="{h}" '
        f'fill="white" stroke="black" stroke-width="2"/>',
        # Usable-area dashed rectangle
        f'  <rect x="{cfg.margin}" y="{cfg.margin}" '
        f'width="{w - 2 * cfg.margin}" height="{h - 2 * cfg.margin}" '
        f'fill="none" stroke="#ccc" stroke-width="0.5" stroke-dasharray="5,5"/>',
    ]

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
    y_first = path[0, 1]  # first track Y
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
    y0 = path[0, 1]
    y1_dim = y0 + cfg.track_spacing
    dim_x = max(path[0, 0] - 30.0, 10.0)
    ax.annotate('', xy=(dim_x, y1_dim), xytext=(dim_x, y0),
                arrowprops=dict(arrowstyle='<->', color='#2266cc', lw=1.5))
    ax.text(dim_x - 12, (y0 + y1_dim) / 2.0,
            f'{cfg.track_spacing:.0f} mm',
            ha='center', va='center', fontsize=8, color='#2266cc',
            rotation=90, fontweight='bold',
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
                        help='Margin from board edge in mm (default: 150)')

    # Pattern
    parser.add_argument('--spacing', type=float, default=100.0,
                        help='Track spacing in mm (default: 100)')
    parser.add_argument('--corner-radius', type=float, default=50.0,
                        help='U-turn corner radius in mm (default: 50)')
    parser.add_argument('--marker-offset', type=float, default=60.0,
                        help='Sync marker distance before turn in mm (default: 60)')
    parser.add_argument('--no-return', action='store_true',
                        help='Omit exterior return path')

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
