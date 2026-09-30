#!/usr/bin/env python3
"""Generate an MJCF scene with a stepped pyramid."""

from __future__ import annotations

import argparse
from pathlib import Path


FIRST_STEP_START_X = 1.0
DEFAULT_OUTPUT = Path(__file__).with_name("stairs.xml")
STEP_COLORS = (
    "0.86 0.26 0.22 1",
    "0.20 0.63 0.87 1",
    "0.22 0.70 0.36 1",
    "0.95 0.66 0.18 1",
    "0.56 0.36 0.82 1",
    "0.92 0.38 0.58 1",
)


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


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def step_material_name(level: int) -> str:
    return f"stairs_step_{level % len(STEP_COLORS) + 1}"


def make_step_materials(step_count: int) -> str:
    material_count = min(step_count, len(STEP_COLORS))
    lines: list[str] = []
    for level, rgba in enumerate(STEP_COLORS[:material_count]):
        lines.append(f'    <material name="{step_material_name(level)}" rgba="{rgba}" />')
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a stepped-pyramid MJCF scene. "
            f"By default the first step starts at x = {fmt(FIRST_STEP_START_X)} m."
        )
    )
    parser.add_argument(
        "--step-height",
        required=True,
        type=positive_float,
        help="Height of one step in meters.",
    )
    parser.add_argument(
        "--step-length",
        required=True,
        type=positive_float,
        help="Horizontal step inset in meters.",
    )
    parser.add_argument(
        "--step-count",
        required=True,
        type=positive_int,
        help="Number of steps in the pyramid.",
    )
    parser.add_argument(
        "--top-platform-size",
        required=True,
        type=positive_float,
        help="Side length of the square top platform in meters.",
    )
    position = parser.add_mutually_exclusive_group()
    position.add_argument(
        "--first-step-dist",
        type=float,
        help=(
            "X coordinate of the first step's outer edge in meters "
            f"(default: {fmt(FIRST_STEP_START_X)})."
        ),
    )
    position.add_argument(
        "--center-pos",
        type=float,
        help="X coordinate of the pyramid center in meters.",
    )
    return parser


def pyramid_bottom_side(
    step_length: float,
    step_count: int,
    top_platform_size: float,
) -> float:
    return top_platform_size + 2.0 * step_length * (step_count - 1)


def resolve_center_x(args: argparse.Namespace) -> float:
    if args.center_pos is not None:
        return args.center_pos
    first_step_dist = (
        args.first_step_dist if args.first_step_dist is not None else FIRST_STEP_START_X
    )
    bottom_side = pyramid_bottom_side(
        args.step_length, args.step_count, args.top_platform_size
    )
    return first_step_dist + bottom_side / 2.0


def make_step_geoms(
    step_height: float,
    step_length: float,
    step_count: int,
    top_platform_size: float,
    center_x: float,
) -> tuple[str, float]:
    bottom_side = pyramid_bottom_side(step_length, step_count, top_platform_size)

    geom_lines: list[str] = []
    for level in range(step_count):
        remaining_rings = step_count - level - 1
        side = top_platform_size + 2.0 * step_length * remaining_rings
        top_height = step_height * (level + 1)
        geom_lines.extend(
            [
                "    <geom",
                f'      name="step_{level + 1}"',
                '      type="box"',
                f'      pos="{fmt(center_x)} 0 {fmt(top_height / 2.0)}"',
                f'      size="{fmt(side / 2.0)} {fmt(side / 2.0)} {fmt(top_height / 2.0)}"',
                '      contype="1"',
                '      conaffinity="1"',
                f'      material="{step_material_name(level)}"',
                "    />",
            ]
        )

    return "\n".join(geom_lines), bottom_side


def render_scene(
    step_height: float,
    step_length: float,
    step_count: int,
    top_platform_size: float,
    center_x: float,
) -> str:
    step_geoms, bottom_side = make_step_geoms(
        step_height=step_height,
        step_length=step_length,
        step_count=step_count,
        top_platform_size=top_platform_size,
        center_x=center_x,
    )
    step_materials = make_step_materials(step_count)

    pyramid_height = step_height * step_count
    ground_half_x = max(15.0, abs(center_x) + bottom_side / 2.0 + 2.0)
    ground_half_y = max(15.0, bottom_side / 2.0 + 2.0)
    statistic_center_z = max(0.1, pyramid_height / 2.0)
    # statistic_extent = max(
    #     ground_half_x,
    #     ground_half_y,
    #     pyramid_height + 2.0,
    # )
    statistic_extent = 2

    return f"""<mujoco model="{{name}} stairs scene">
  <include file="{{path}}" />

  <statistic center="{fmt(center_x)} 0 {fmt(statistic_center_z)}" extent="{fmt(statistic_extent)}" meansize="0.04" />

  <visual>
    <headlight diffuse="0.0 0.0 0.0" ambient="0.0 0.0 0.0" specular="0.0 0.0 0.0" />
    <global azimuth="220" elevation="-10" />
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
{step_materials}
  </asset>

  <worldbody>
    <light
      name="sun"
      directional="true"
      dir="-0.5 -0.4 -1"
      diffuse="0.4 0.4 0.4"
      specular="0.2 0.2 0.2"
      ambient="0.01 0.01 0.01"
      castshadow="true"
    />

    <light
      name="spotlight1"
      mode="targetbodycom"
      target="base"
      diffuse="0.2 0.2 0.2"
      specular="0.1 0.1 0.1"
      pos="0 -10 4"
      cutoff="10"
    />

    <light
      name="spotlight2"
      mode="targetbodycom"
      directional="true"
      target="base"
      diffuse="0.2 0.2 0.2"
      specular="0.1 0.1 0.1"
      pos="10 0 4"
      dir="1 0 0"
      cutoff="10"
      castshadow="false"
    />

    <geom name="ground" type="plane" pos="0 0 0" size="{fmt(ground_half_x)} {fmt(ground_half_y)} 0.1" material="grid"/>

{step_geoms}
  </worldbody>
</mujoco>
"""


def render_generation_comment(
    step_height: float,
    step_length: float,
    step_count: int,
    top_platform_size: float,
    center_x: float,
) -> str:
    bottom_side = pyramid_bottom_side(step_length, step_count, top_platform_size)
    return "\n".join(
        [
            "<!-- Generated by generate_stairs.py with arguments:",
            f"  step-height {fmt(step_height)}",
            f"  step-length {fmt(step_length)}",
            f"  step-count {step_count}",
            f"  top-platform-size {fmt(top_platform_size)}",
            f"  first-step-dist {fmt(center_x - bottom_side / 2.0)}",
            f"  center-pos {fmt(center_x)}",
            "-->",
        ]
    )


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    center_x = resolve_center_x(args)

    scene_xml = render_scene(
        step_height=args.step_height,
        step_length=args.step_length,
        step_count=args.step_count,
        top_platform_size=args.top_platform_size,
        center_x=center_x,
    )
    generation_comment = render_generation_comment(
        step_height=args.step_height,
        step_length=args.step_length,
        step_count=args.step_count,
        top_platform_size=args.top_platform_size,
        center_x=center_x,
    )
    xml = f"{generation_comment}\n{scene_xml}"

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_OUTPUT.write_text(xml, encoding="utf-8")
    print(f"Generated {DEFAULT_OUTPUT}")


if __name__ == "__main__":
    main()
