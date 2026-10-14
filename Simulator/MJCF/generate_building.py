#!/usr/bin/env python3
"""Generate an MJCF two-level indoor scene: rooms, corridor, mezzanine with a ramp
and stairs, step-over boxes and free-falling boards."""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

from generate_boards import (
    DEFAULT_DENSITY,
    DEFAULT_LENGTH_VARIATION,
    DEFAULT_THICKNESS_VARIATION,
    DEFAULT_WIDTH_VARIATION,
    axis_half_extent,
    board_inertia,
    quaternion_to_matrix,
    sample_dimension,
    vertical_half_extent,
)
from generate_stairs import make_step_materials, step_material_name


DEFAULT_OUTPUT = Path(__file__).with_name("building.xml")

# Plan layout, meters. The origin lies in the free zone of the hall.
X_MIN, X_MAX = -8.5, 8.0
Y_MIN, Y_MAX = -4.0, 4.0
WALL_THICKNESS = 0.1
PARTITION_X = 3.0             # hall | rooms and corridor
ROOM_DIVIDER_X = 5.5          # room A | room B
CORRIDOR_HALF_WIDTH = 0.6
HALL_DOOR_Y = -2.5            # hall -> south room
NORTH_DOORS_X = (4.25, 6.75)  # corridor -> rooms A, B
SOUTH_DOOR_X = 6.75           # corridor -> south room

MEZZANINE_X = (-4.0, -1.5)
MEZZANINE_Y = (-1.0, 4.0)
DECK_THICKNESS = 0.06
RAMP_Y = (2.8, 4.0)           # along the north wall, descends towards +X
RAMP_THICKNESS = 0.04
PARAPET_HEIGHT = 0.4
PARAPET_THICKNESS = 0.05
COLUMN_HALF_SIZE = 0.06
COLUMNS = ((-1.5, -1.0), (-1.5, 0.9), (-4.0, 0.5), (-4.0, 2.3))  # under the open deck edges
STAIRS_Y = (-1.0, 0.5)        # at the west edge of the deck, descends towards -X

# name: (x, y, half length, half width, height, z base: "floor" | "deck")
BOXES = {
    "box_hall": (-0.6, -3.2, 0.3, 0.2, 0.3, "floor"),
    "box_mezzanine": (-3.3, 1.5, 0.25, 0.25, 0.3, "deck"),
    "box_room_a": (4.6, 3.0, 0.25, 0.3, 0.4, "floor"),
    "box_room_b": (7.2, 2.0, 0.2, 0.4, 0.25, "floor"),
    "box_south_1": (5.0, -2.8, 0.4, 0.25, 0.5, "floor"),
    "box_south_2": (7.3, -3.2, 0.25, 0.25, 0.2, "floor"),
}
HALL_COLUMN = (0.8, 2.0, 0.15)  # x, y, half size
# Not in the map: a box on the straight line from the origin to the corridor doorway, for
# the navigation obstacle-avoidance test (--obstacle). x, y, half size, height
NAV_OBSTACLE = (1.6, 0.0, 0.2, 0.4)

# Step-over strips as in generate_boxes.py, from the outer wall to the stairs side
# panel, spread evenly along the stairs; heights grow from east to west.
BOX_STRIP_HALF_X = 0.2
BOX_STRIP_COUNTS = {"boxes_north": 3, "boxes_south": 4}
MIN_BOX_STRIP_GAP = 0.3

# Free-falling boards as in generate_boards.py, in front of the corridor entrance.
BOARDS_X = (2.2, 4.3)
BOARDS_HALF_Y = 0.45          # within the doorway and the corridor
BOARD_LENGTH = 0.6
BOARD_WIDTH = 0.2
BOARD_DROP_HEIGHT = 0.3       # clearance from the lowest board point to the floor
BOARD_MAX_TILT_DEG = 20.0     # boards fall nearly flat, a steeper tilt may leave them on edge
BOARD_PLACE_ATTEMPTS = 100    # free spots tried before a board is dropped onto others
BOARD_STACK_GAP = 0.02        # between a board dropped onto others and their tops

MIN_CLEARANCE = 0.45          # under the mezzanine
MIN_DOOR_WIDTH = 0.8
MIN_RAMP_EXIT = 1.2           # free floor between the ramp or stairs foot and the wall
MIN_HEADROOM = 1.0            # above the mezzanine deck


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def slope_angle(value: str) -> float:
    parsed = float(value)
    if parsed <= 0 or parsed >= 89:
        raise argparse.ArgumentTypeError("angle must be between 0 and 89 degrees")
    return parsed


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an MJCF two-level building "
            f"{fmt(X_MAX - X_MIN)} x {fmt(Y_MAX - Y_MIN)} m: hall with a mezzanine "
            "reachable by a ramp and stairs, corridor, three rooms, obstacles "
            "and a translucent ceiling."
        )
    )
    parser.add_argument("--wall-height", type=positive_float, default=2.0,
                        help="Wall height and ceiling level in meters (default: 2.0).")
    parser.add_argument("--mezzanine-height", type=positive_float, default=0.6,
                        help="Height of the mezzanine deck top in meters (default: 0.6).")
    parser.add_argument("--ramp-angle", type=slope_angle, default=12.0,
                        help="Ramp slope angle in degrees (default: 12).")
    parser.add_argument("--stair-step-height", type=positive_float, default=0.05,
                        help="Stair step height in meters, rounded down to divide the mezzanine "
                             "height evenly (default: 0.05).")
    parser.add_argument("--stair-step-depth", type=positive_float, default=0.25,
                        help="Stair step depth in meters (default: 0.25).")
    parser.add_argument("--door-width", type=positive_float, default=1.0,
                        help="Width of all doorways in meters (default: 1.0).")
    parser.add_argument("--box-first-height", type=positive_float, default=0.02,
                        help="Height of the easternmost step-over box in each zone in meters (default: 0.02).")
    parser.add_argument("--box-last-height", type=positive_float, default=0.08,
                        help="Height of the westernmost step-over box in each zone in meters (default: 0.08).")
    parser.add_argument("--board-count", type=positive_int, default=4,
                        help="Number of free-falling boards at the corridor entrance (default: 4).")
    parser.add_argument("--board-thickness", type=positive_float, default=0.04,
                        help="Average board thickness in meters (default: 0.04).")
    parser.add_argument("--seed", type=int, default=7,
                        help="Random seed of the board sizes and poses (default: 7).")
    parser.add_argument("--ceiling", action=argparse.BooleanOptionalAction, default=True,
                        help="Add a translucent ceiling seen by the lidar (default: on).")
    parser.add_argument("--obstacle", action="store_true",
                        help="Add a box missing from the navigation map in the hall, on the way "
                             "from the origin to the corridor (default: off).")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help=f"Output MJCF path (default: {DEFAULT_OUTPUT.name} next to this script).")
    return parser


def ramp_run(height: float, angle_deg: float) -> float:
    return height / math.tan(math.radians(angle_deg))


def stair_step_count(height: float, step_height: float) -> int:
    return math.ceil(height / step_height - 1e-9)


def stairs_length(height: float, step_height: float, step_depth: float) -> float:
    # The top riser leads onto the deck, so there is one tread less than risers.
    return (stair_step_count(height, step_height) - 1) * step_depth


def box_strip_spacing(stairs_len: float, count: int) -> float:
    return (stairs_len - 2.0 * BOX_STRIP_HALF_X) / (count - 1) if count > 1 else 0.0


def validate(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    clearance = args.mezzanine_height - DECK_THICKNESS
    ramp_end_x = MEZZANINE_X[1] + ramp_run(args.mezzanine_height, args.ramp_angle)
    if clearance < MIN_CLEARANCE:
        parser.error(f"clearance under the mezzanine {fmt(clearance)} m < {fmt(MIN_CLEARANCE)} m")
    if not MIN_DOOR_WIDTH <= args.door_width <= 2.0 * CORRIDOR_HALF_WIDTH:
        parser.error(f"door width must be in [{fmt(MIN_DOOR_WIDTH)}, {fmt(2.0 * CORRIDOR_HALF_WIDTH)}] m")
    if ramp_end_x > PARTITION_X - MIN_RAMP_EXIT:
        parser.error(f"ramp ends at x = {fmt(ramp_end_x)} m, too close to the partition at x = {fmt(PARTITION_X)} m")
    if args.wall_height < args.mezzanine_height + MIN_HEADROOM:
        parser.error(f"wall height must be at least mezzanine height + {fmt(MIN_HEADROOM)} m")
    if args.stair_step_height > args.mezzanine_height:
        parser.error("stair step height must not exceed the mezzanine height")
    stairs_foot_x = MEZZANINE_X[0] - stairs_length(args.mezzanine_height, args.stair_step_height,
                                                   args.stair_step_depth)
    if stairs_foot_x < X_MIN + MIN_RAMP_EXIT:
        parser.error(f"stairs end at x = {fmt(stairs_foot_x)} m, too close to the wall at x = {fmt(X_MIN)} m")
    strips_gap = box_strip_spacing(MEZZANINE_X[0] - stairs_foot_x, max(BOX_STRIP_COUNTS.values())) \
        - 2.0 * BOX_STRIP_HALF_X
    if strips_gap < MIN_BOX_STRIP_GAP:
        parser.error(f"gap between step-over boxes {fmt(strips_gap)} m < {fmt(MIN_BOX_STRIP_GAP)} m, "
                     "make the stairs longer")


def box_geom(name: str, center: tuple[float, float, float], half: tuple[float, float, float],
             material: str, euler: tuple[float, float, float] | None = None, collide: bool = True) -> str:
    attrs = [
        f'name="{name}"',
        'type="box"',
        f'pos="{" ".join(fmt(v) for v in center)}"',
        f'size="{" ".join(fmt(v) for v in half)}"',
    ]
    if euler is not None:
        attrs.append(f'euler="{" ".join(fmt(v) for v in euler)}"')
    contact = "1" if collide else "0"
    attrs += [f'contype="{contact}"', f'conaffinity="{contact}"', f'material="{material}"']
    return f"    <geom {' '.join(attrs)}/>"


def wall_geoms(name: str, along_x: bool, fixed: float, start: float, end: float,
               height: float, doors: tuple[float, ...] = (), door_width: float = 0.0) -> list[str]:
    """Straight wall from start to end at the fixed coordinate, with doorways centered at doors."""
    edges = [start]
    for center in sorted(doors):
        edges += [center - door_width / 2.0, center + door_width / 2.0]
    edges.append(end)

    geoms = []
    for i, (a, b) in enumerate(zip(edges[::2], edges[1::2])):
        mid, half = (a + b) / 2.0, (b - a) / 2.0
        center = (mid, fixed, height / 2.0) if along_x else (fixed, mid, height / 2.0)
        size = (half, WALL_THICKNESS / 2.0, height / 2.0) if along_x else (WALL_THICKNESS / 2.0, half, height / 2.0)
        geoms.append(box_geom(f"{name}_{i}", center, size, "wall"))
    return geoms


def make_walls(height: float, door_width: float) -> list[str]:
    outer = WALL_THICKNESS / 2.0
    x0, x1 = X_MIN - WALL_THICKNESS, X_MAX + WALL_THICKNESS
    return [
        *wall_geoms("wall_south", True, Y_MIN - outer, x0, x1, height),
        *wall_geoms("wall_north", True, Y_MAX + outer, x0, x1, height),
        *wall_geoms("wall_west", False, X_MIN - outer, Y_MIN, Y_MAX, height),
        *wall_geoms("wall_east", False, X_MAX + outer, Y_MIN, Y_MAX, height),
        *wall_geoms("wall_partition", False, PARTITION_X, Y_MIN, Y_MAX, height, (HALL_DOOR_Y, 0.0), door_width),
        *wall_geoms("wall_corridor_north", True, CORRIDOR_HALF_WIDTH, PARTITION_X, X_MAX, height,
                    NORTH_DOORS_X, door_width),
        *wall_geoms("wall_corridor_south", True, -CORRIDOR_HALF_WIDTH, PARTITION_X, X_MAX, height,
                    (SOUTH_DOOR_X,), door_width),
        *wall_geoms("wall_rooms", False, ROOM_DIVIDER_X, CORRIDOR_HALF_WIDTH, Y_MAX, height),
    ]


def make_mezzanine(height: float, ramp_angle_deg: float) -> list[str]:
    (x0, x1), (y0, y1) = MEZZANINE_X, MEZZANINE_Y
    deck_bottom = height - DECK_THICKNESS
    parapet_z = height + PARAPET_HEIGHT / 2.0
    parapet_half_t = PARAPET_THICKNESS / 2.0

    geoms = [
        box_geom("mezzanine_deck", ((x0 + x1) / 2.0, (y0 + y1) / 2.0, height - DECK_THICKNESS / 2.0),
                 ((x1 - x0) / 2.0, (y1 - y0) / 2.0, DECK_THICKNESS / 2.0), "deck"),
        box_geom("parapet_south", ((x0 + x1) / 2.0, y0 + parapet_half_t, parapet_z),
                 ((x1 - x0) / 2.0, parapet_half_t, PARAPET_HEIGHT / 2.0), "parapet"),
        box_geom("parapet_east", (x1 - parapet_half_t, (y0 + RAMP_Y[0]) / 2.0, parapet_z),
                 (parapet_half_t, (RAMP_Y[0] - y0) / 2.0, PARAPET_HEIGHT / 2.0), "parapet"),
        box_geom("parapet_west", (x0 + parapet_half_t, (STAIRS_Y[1] + y1) / 2.0, parapet_z),
                 (parapet_half_t, (y1 - STAIRS_Y[1]) / 2.0, PARAPET_HEIGHT / 2.0), "parapet"),
    ]
    for i, (x, y) in enumerate(COLUMNS):
        cx = min(max(x, x0 + COLUMN_HALF_SIZE), x1 - COLUMN_HALF_SIZE)
        cy = min(max(y, y0 + COLUMN_HALF_SIZE), y1 - COLUMN_HALF_SIZE)
        geoms.append(box_geom(f"column_mezzanine_{i}", (cx, cy, deck_bottom / 2.0),
                              (COLUMN_HALF_SIZE, COLUMN_HALF_SIZE, deck_bottom / 2.0), "column"))

    # Ramp slab: its top surface goes from (x1, height) down to (x1 + run, 0).
    angle = math.radians(ramp_angle_deg)
    run = ramp_run(height, ramp_angle_deg)
    ramp_half_w = (RAMP_Y[1] - RAMP_Y[0]) / 2.0
    ramp_y = (RAMP_Y[0] + RAMP_Y[1]) / 2.0
    half_t = RAMP_THICKNESS / 2.0
    geoms.append(box_geom(
        "ramp",
        (x1 + run / 2.0 - math.sin(angle) * half_t, ramp_y, height / 2.0 - math.cos(angle) * half_t),
        (math.hypot(run, height) / 2.0, ramp_half_w, half_t),
        "deck",
        euler=(0.0, ramp_angle_deg, 0.0),
    ))
    # Side panel on the open side of the ramp and a wall closing the space under it.
    side_height = height + PARAPET_HEIGHT
    geoms.append(box_geom("ramp_side", (x1 + run / 2.0, RAMP_Y[0] - parapet_half_t, side_height / 2.0),
                          (run / 2.0, parapet_half_t, side_height / 2.0), "parapet"))
    geoms.append(box_geom("ramp_back", (x1 + parapet_half_t, ramp_y, deck_bottom / 2.0),
                          (parapet_half_t, ramp_half_w, deck_bottom / 2.0), "parapet"))
    return geoms


def make_stairs(height: float, step_height: float, step_depth: float) -> list[str]:
    """Solid steps from the floor to the west edge of the deck, with side panels."""
    edge_x = MEZZANINE_X[0]
    count = stair_step_count(height, step_height)
    rise = height / count
    half_w = (STAIRS_Y[1] - STAIRS_Y[0]) / 2.0
    stairs_y = (STAIRS_Y[0] + STAIRS_Y[1]) / 2.0

    geoms = []
    for k in range(1, count):
        top = (count - k) * rise
        geoms.append(box_geom(f"stairs_step_{count - k}", (edge_x - (k - 0.5) * step_depth, stairs_y, top / 2.0),
                              (step_depth / 2.0, half_w, top / 2.0), step_material_name(count - k - 1)))

    length = (count - 1) * step_depth
    side_height = height + PARAPET_HEIGHT
    parapet_half_t = PARAPET_THICKNESS / 2.0
    for name, y in (("south", STAIRS_Y[0] - parapet_half_t), ("north", STAIRS_Y[1] + parapet_half_t)):
        geoms.append(box_geom(f"stairs_side_{name}", (edge_x - length / 2.0, y, side_height / 2.0),
                              (length / 2.0, parapet_half_t, side_height / 2.0), "parapet"))
    return geoms


def make_box_strips(first_height: float, last_height: float, stairs_len: float) -> list[str]:
    side_offset = PARAPET_THICKNESS
    zones_y = {
        "boxes_north": (STAIRS_Y[1] + side_offset, Y_MAX),
        "boxes_south": (Y_MIN, STAIRS_Y[0] - side_offset),
    }
    east_x = MEZZANINE_X[0] - BOX_STRIP_HALF_X
    geoms = []
    for zone, count in BOX_STRIP_COUNTS.items():
        y0, y1 = zones_y[zone]
        spacing = box_strip_spacing(stairs_len, count)
        step = (last_height - first_height) / (count - 1) if count > 1 else 0.0
        for i in range(count):
            height = first_height + i * step
            geoms.append(box_geom(f"{zone}_{i + 1}", (east_x - i * spacing, (y0 + y1) / 2.0, height / 2.0),
                                  (BOX_STRIP_HALF_X, (y1 - y0) / 2.0, height / 2.0), "obstacle"))
    return geoms


def flat_quaternion(rng: random.Random, max_tilt_deg: float) -> tuple[float, float, float, float]:
    """Random yaw, then a tilt of up to max_tilt_deg about a random horizontal axis."""
    yaw = rng.uniform(-math.pi, math.pi)
    tilt = math.radians(rng.uniform(0.0, max_tilt_deg))
    axis = rng.uniform(-math.pi, math.pi)
    # q_tilt * q_yaw, q_yaw = (cos(yaw/2), 0, 0, sin(yaw/2)), the tilt axis lies in the XY plane
    tw, tx, ty = math.cos(tilt / 2.0), math.cos(axis) * math.sin(tilt / 2.0), math.sin(axis) * math.sin(tilt / 2.0)
    yw, yz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (tw * yw, tx * yw + ty * yz, ty * yw - tx * yz, tw * yz)


def footprint(half_sizes: tuple[float, float, float],
              rotation: tuple[tuple[float, float, float], ...]) -> list[tuple[float, float]]:
    """XY generators of the box projection: the footprint is their zonotope around the center."""
    return [(rotation[0][i] * h, rotation[1][i] * h) for i, h in enumerate(half_sizes)]


def footprints_overlap(a: tuple[float, float], gens_a: list[tuple[float, float]],
                       b: tuple[float, float], gens_b: list[tuple[float, float]]) -> bool:
    """Separating axis test of two zonotopes, the candidate axes are normal to their generators."""
    for gx, gy in gens_a + gens_b:
        norm = math.hypot(gx, gy)
        if norm < 1e-9:
            continue
        nx, ny = -gy / norm, gx / norm
        reach = sum(abs(x * nx + y * ny) for x, y in gens_a + gens_b)
        if abs((b[0] - a[0]) * nx + (b[1] - a[1]) * ny) > reach:
            return False
    return True


def make_boards(count: int, thickness: float, rng: random.Random) -> list[str] | None:
    """Free-falling boards with a random yaw anywhere in the zone; a board without a free spot
    is dropped from above the boards under it. None if a board does not fit the zone."""
    placed = []  # (center xy, footprint, top z)
    lines = []
    for i in range(count):
        half_sizes = (sample_dimension(rng, BOARD_LENGTH, DEFAULT_LENGTH_VARIATION) / 2.0,
                      sample_dimension(rng, BOARD_WIDTH, DEFAULT_WIDTH_VARIATION) / 2.0,
                      sample_dimension(rng, thickness, DEFAULT_THICKNESS_VARIATION) / 2.0)
        quat = flat_quaternion(rng, BOARD_MAX_TILT_DEG)
        rotation = quaternion_to_matrix(quat)
        half_x = axis_half_extent(half_sizes, rotation, axis_index=0)
        half_y = axis_half_extent(half_sizes, rotation, axis_index=1)
        half_z = vertical_half_extent(half_sizes, rotation)
        slack_y = BOARDS_HALF_Y - half_y
        if BOARDS_X[1] - BOARDS_X[0] < 2.0 * half_x or slack_y < 0.0:
            return None

        gens = footprint(half_sizes, rotation)
        for _ in range(BOARD_PLACE_ATTEMPTS):
            xy = (rng.uniform(BOARDS_X[0] + half_x, BOARDS_X[1] - half_x), rng.uniform(-slack_y, slack_y))
            below = [top for center, other, top in placed if footprints_overlap(xy, gens, center, other)]
            if not below:
                break
        bottom = max([BOARD_DROP_HEIGHT] + [top + BOARD_STACK_GAP for top in below])
        z = bottom + half_z
        placed.append((xy, gens, z + half_z))

        mass, inertia = board_inertia(half_sizes, DEFAULT_DENSITY)
        lines += [
            f'    <body name="board_{i + 1}" pos="{fmt(xy[0])} {fmt(xy[1])} {fmt(z)}" quat="{" ".join(fmt(v) for v in quat)}">',
            "      <freejoint/>",
            f'      <inertial pos="0 0 0" mass="{fmt(mass)}" diaginertia="{" ".join(fmt(v) for v in inertia)}"/>',
            f'      <geom type="box" size="{" ".join(fmt(v) for v in half_sizes)}" contype="1" conaffinity="1" material="board"/>',
            "    </body>",
        ]
    return lines


def make_obstacles(mezzanine_height: float) -> list[str]:
    geoms = []
    for name, (x, y, hx, hy, h, base) in BOXES.items():
        z0 = mezzanine_height if base == "deck" else 0.0
        geoms.append(box_geom(name, (x, y, z0 + h / 2.0), (hx, hy, h / 2.0), "obstacle"))
    return geoms


def make_hall_column(wall_height: float) -> str:
    x, y, half = HALL_COLUMN
    return box_geom("column_hall", (x, y, wall_height / 2.0), (half, half, wall_height / 2.0), "column")


def make_nav_obstacle() -> str:
    x, y, half, height = NAV_OBSTACLE
    return box_geom("obstacle", (x, y, height / 2.0), (half, half, height / 2.0), "obstacle")


def make_ceiling(wall_height: float) -> str:
    half_t = WALL_THICKNESS / 2.0
    return box_geom(
        "ceiling",
        ((X_MIN + X_MAX) / 2.0, (Y_MIN + Y_MAX) / 2.0, wall_height + half_t),
        ((X_MAX - X_MIN) / 2.0 + WALL_THICKNESS, (Y_MAX - Y_MIN) / 2.0 + WALL_THICKNESS, half_t),
        "ceiling",
        collide=False,
    )


def render_scene(wall_height: float, mezzanine_height: float, ramp_angle: float,
                 stair_step_height: float, stair_step_depth: float, door_width: float,
                 box_first_height: float, box_last_height: float, boards: list[str], ceiling: bool,
                 obstacle: bool = False) -> str:
    geoms = [
        *make_walls(wall_height, door_width),
        *make_mezzanine(mezzanine_height, ramp_angle),
        *make_stairs(mezzanine_height, stair_step_height, stair_step_depth),
        *make_obstacles(mezzanine_height),
        *make_box_strips(box_first_height, box_last_height,
                         stairs_length(mezzanine_height, stair_step_height, stair_step_depth)),
        make_hall_column(wall_height),
    ]
    if obstacle:
        geoms.append(make_nav_obstacle())
    if ceiling:
        geoms.append(make_ceiling(wall_height))
    building_geoms = "\n".join(geoms + boards)

    ground_half_x = max(15.0, abs(X_MIN) + 2.0, abs(X_MAX) + 2.0)
    ground_half_y = max(15.0, abs(Y_MIN) + 2.0, abs(Y_MAX) + 2.0)

    # Shadows are off: the ceiling would otherwise darken the whole interior.
    return f"""<mujoco model="{{name}} building scene">
  <include file="{{path}}" />

  <statistic center="0 0 0.3" extent="2" meansize="0.04" />

  <visual>
    <headlight diffuse="0.3 0.3 0.3" ambient="0.1 0.1 0.1" specular="0.0 0.0 0.0" />
    <global azimuth="220" elevation="-30" />
    <quality shadowsize="8192" />
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="edge" markrgb=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="15 15" reflectance=".05"/>
  </asset>

  <asset>
    <texture
      type="skybox"
      builtin="gradient"
      rgb1="0.2 0.2 0.9"
      rgb2="0.1 0.6 0.2"
      width="512"
      height="3072"
    />
    <material name="wall" rgba="0.75 0.75 0.70 1" />
    <material name="deck" rgba="0.55 0.40 0.20 1" />
    <material name="parapet" rgba="0.45 0.30 0.15 1" />
    <material name="column" rgba="0.50 0.50 0.55 1" />
    <material name="obstacle" rgba="0.70 0.45 0.20 1" />
    <material name="board" rgba="0.63 0.66 0.72 1" />
{make_step_materials(stair_step_count(mezzanine_height, stair_step_height))}
    <material name="ceiling" rgba="0.80 0.80 0.85 0.1" />
  </asset>

  <worldbody>
    <light
      name="sun"
      directional="true"
      dir="-0.5 -0.4 -1"
      diffuse="0.4 0.4 0.4"
      specular="0.2 0.2 0.2"
      ambient="0.2 0.2 0.2"
      castshadow="false"
    />

    <light
      name="spotlight1"
      mode="targetbodycom"
      target="base"
      diffuse="0.2 0.2 0.2"
      specular="0.1 0.1 0.1"
      pos="0 -3 1.8"
      cutoff="30"
      castshadow="false"
    />

    <geom name="ground" type="plane" pos="0 0 0" size="{fmt(ground_half_x)} {fmt(ground_half_y)} 0.1" material="grid"/>

{building_geoms}
  </worldbody>
</mujoco>
"""


def render_generation_comment(args: argparse.Namespace) -> str:
    step_count = stair_step_count(args.mezzanine_height, args.stair_step_height)
    return "\n".join(
        [
            "<!-- Generated by generate_building.py with arguments:",
            f"  wall-height {fmt(args.wall_height)}",
            f"  mezzanine-height {fmt(args.mezzanine_height)}",
            f"  ramp-angle {fmt(args.ramp_angle)}",
            f"  stair-step-height {fmt(args.stair_step_height)}"
            f" (actual {fmt(args.mezzanine_height / step_count)}, {step_count} risers)",
            f"  stair-step-depth {fmt(args.stair_step_depth)}",
            f"  door-width {fmt(args.door_width)}",
            f"  box-first-height {fmt(args.box_first_height)}",
            f"  box-last-height {fmt(args.box_last_height)}",
            f"  board-count {args.board_count}",
            f"  board-thickness {fmt(args.board_thickness)}",
            f"  seed {args.seed}",
            f"  {'ceiling' if args.ceiling else 'no-ceiling'}",
            *(["  obstacle"] if args.obstacle else []),
            "-->",
        ]
    )


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    validate(parser, args)
    boards = make_boards(args.board_count, args.board_thickness, random.Random(args.seed))
    if boards is None:
        parser.error("a board does not fit at the corridor entrance, reduce --board-thickness")
    scene = render_scene(
        wall_height=args.wall_height,
        mezzanine_height=args.mezzanine_height,
        ramp_angle=args.ramp_angle,
        stair_step_height=args.stair_step_height,
        stair_step_depth=args.stair_step_depth,
        door_width=args.door_width,
        box_first_height=args.box_first_height,
        box_last_height=args.box_last_height,
        boards=boards,
        ceiling=args.ceiling,
        obstacle=args.obstacle,
    )
    xml = f"{render_generation_comment(args)}\n{scene}"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(xml, encoding="utf-8")
    print(f"Generated {args.output}")


if __name__ == "__main__":
    main()
