#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from manim import *


Point2D = tuple[float, float]

ACTIVE_SCENE_SPEC: "SceneSpec | None" = None


@dataclass(frozen=True)
class GuardSpec:
    point: Point2D | None
    path: tuple[Point2D, ...]
    color: str
    cycles: float
    ping_pong: bool
    radius: float
    trail: bool
    label: str | None


@dataclass(frozen=True)
class SceneSpec:
    polygon: tuple[Point2D, ...]
    guards: tuple[GuardSpec, ...]
    title: str | None
    duration: float
    hold_time: float
    background_color: str
    polygon_fill: str
    polygon_stroke: str
    track_color: str
    media_dir: str
    output_file: str
    quality: str
    fps: int
    preview: bool


def parse_point(raw: Any) -> Point2D:
    if not isinstance(raw, (list, tuple)) or len(raw) < 2:
        raise ValueError(f"Expected a 2D point, got {raw!r}")
    return float(raw[0]), float(raw[1])


def parse_point_lines(text: str) -> list[list[Point2D]]:
    rings: list[list[Point2D]] = []
    curr: list[Point2D] = []

    for line in text.splitlines():
        text_line = line.split("#", 1)[0].strip()
        if not text_line:
            if curr:
                rings.append(curr)
                curr = []
            continue

        text_line = text_line.replace(",", " ")
        parts = text_line.split()
        if len(parts) < 2:
            continue
        curr.append((float(parts[0]), float(parts[1])))

    if curr:
        rings.append(curr)

    return rings


def load_polygon_points(path: Path) -> tuple[Point2D, ...]:
    rings = parse_point_lines(path.read_text())
    if not rings or len(rings[0]) < 3:
        raise ValueError(f"Polygon file must contain at least three points: {path}")
    return tuple(rings[0])


def load_scene_spec(path: Path, cli_args: argparse.Namespace) -> SceneSpec:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError("Scene file must contain a JSON object.")

    if "polygon" in data:
        polygon = tuple(parse_point(pt) for pt in data["polygon"])
    elif "polygon_file" in data:
        polygon_path = (path.parent / data["polygon_file"]).resolve()
        polygon = load_polygon_points(polygon_path)
    else:
        raise ValueError("Scene file must provide either 'polygon' or 'polygon_file'.")

    guards_raw = data.get("guards", [])
    guards: list[GuardSpec] = []
    for idx, guard_raw in enumerate(guards_raw):
        if not isinstance(guard_raw, dict):
            raise ValueError(f"Guard {idx} must be a JSON object.")

        point = guard_raw.get("point")
        path_points = guard_raw.get("path", guard_raw.get("points"))

        if point is not None and path_points is not None:
            raise ValueError(f"Guard {idx} should define either 'point' or 'path', not both.")

        if point is not None:
            guard_point = parse_point(point)
            guards.append(
                GuardSpec(
                    point=guard_point,
                    path=(),
                    color=str(guard_raw.get("color", "#7dd3fc")),
                    cycles=0.0,
                    ping_pong=False,
                    radius=float(guard_raw.get("radius", 0.09)),
                    trail=False,
                    label=guard_raw.get("label", f"G{idx + 1}"),
                )
            )
            continue

        if path_points is None:
            raise ValueError(f"Guard {idx} must define either 'point' or 'path'.")

        parsed_path = tuple(parse_point(pt) for pt in path_points)
        if len(parsed_path) < 2:
            raise ValueError(f"Guard {idx} path must contain at least two points.")

        guards.append(
            GuardSpec(
                point=None,
                path=parsed_path,
                color=str(guard_raw.get("color", "#f59e0b")),
                cycles=float(guard_raw.get("cycles", 2.0)),
                ping_pong=bool(guard_raw.get("ping_pong", True)),
                radius=float(guard_raw.get("radius", 0.09)),
                trail=bool(guard_raw.get("trail", True)),
                label=guard_raw.get("label", f"G{idx + 1}"),
            )
        )

    output_file = cli_args.output_file or str(data.get("output_file", path.stem))
    media_dir = str(Path(cli_args.media_dir or data.get("media_dir", "temp/manim")).resolve())
    quality = str(cli_args.quality or data.get("quality", "medium_quality"))
    fps = int(cli_args.fps or data.get("fps", 30))

    return SceneSpec(
        polygon=polygon,
        guards=tuple(guards),
        title=data.get("title"),
        duration=float(data.get("duration", 10.0)),
        hold_time=float(data.get("hold_time", 0.75)),
        background_color=str(data.get("background_color", "#0b1320")),
        polygon_fill=str(data.get("polygon_fill", "#111b2c")),
        polygon_stroke=str(data.get("polygon_stroke", "#e2e8f0")),
        track_color=str(data.get("track_color", "#94a3b8")),
        media_dir=media_dir,
        output_file=output_file,
        quality=quality,
        fps=fps,
        preview=bool(cli_args.preview),
    )


class CoordinateMapper:
    def __init__(self, points: list[Point2D], target_width: float = 11.0, target_height: float = 6.0):
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        self.cx = (min(xs) + max(xs)) / 2.0
        self.cy = (min(ys) + max(ys)) / 2.0
        span_x = max(max(xs) - min(xs), 1e-6)
        span_y = max(max(ys) - min(ys), 1e-6)
        self.scale = min(target_width / span_x, target_height / span_y)

    def __call__(self, pt: Point2D) -> np.ndarray:
        return np.array([(pt[0] - self.cx) * self.scale, (pt[1] - self.cy) * self.scale, 0.0])


def motion_progress(value: float, ping_pong: bool) -> float:
    if ping_pong:
        folded = value % 2.0
        return folded if folded <= 1.0 else 2.0 - folded
    return value % 1.0


def point_distance(p1: Point2D, p2: Point2D) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def cross_2d(a: Point2D, b: Point2D) -> float:
    return a[0] * b[1] - a[1] * b[0]


def subtract_points(a: Point2D, b: Point2D) -> Point2D:
    return (a[0] - b[0], a[1] - b[1])


def ray_segment_intersection(origin: Point2D, angle: float, a: Point2D, b: Point2D, eps: float = 1e-9) -> tuple[float, Point2D] | None:
    direction = (math.cos(angle), math.sin(angle))
    segment = subtract_points(b, a)
    offset = subtract_points(a, origin)
    denom = cross_2d(direction, segment)
    if abs(denom) < eps:
        return None

    t = cross_2d(offset, segment) / denom
    u = cross_2d(offset, direction) / denom
    if t < -eps or u < -eps or u > 1.0 + eps:
        return None

    hit = (origin[0] + t * direction[0], origin[1] + t * direction[1])
    return t, hit


def clean_polygon_points(points: list[Point2D], tol: float = 1e-5) -> list[Point2D]:
    cleaned: list[Point2D] = []
    for pt in points:
        if cleaned and point_distance(cleaned[-1], pt) <= tol:
            continue
        cleaned.append(pt)

    if len(cleaned) > 1 and point_distance(cleaned[0], cleaned[-1]) <= tol:
        cleaned.pop()

    return cleaned


def visibility_polygon_from_point(point: Point2D, polygon: tuple[Point2D, ...], eps_angle: float = 1e-5) -> list[Point2D]:
    angles: list[float] = []
    for vertex in polygon:
        base = math.atan2(vertex[1] - point[1], vertex[0] - point[0])
        angles.extend((base - eps_angle, base, base + eps_angle))

    hits: list[tuple[float, Point2D]] = []
    for angle in angles:
        closest_t: float | None = None
        closest_hit: Point2D | None = None
        for idx in range(len(polygon)):
            a = polygon[idx]
            b = polygon[(idx + 1) % len(polygon)]
            result = ray_segment_intersection(point, angle, a, b)
            if result is None:
                continue
            t, hit = result
            if closest_t is None or t < closest_t:
                closest_t = t
                closest_hit = hit

        if closest_hit is not None:
            hits.append((angle, closest_hit))

    hits.sort(key=lambda item: item[0])
    return clean_polygon_points([pt for _, pt in hits])


def make_guard_visual(center: np.ndarray, color: str, radius: float, label: str | None) -> VGroup:
    halo = Circle(radius=radius * 2.2, stroke_width=0, fill_color=color, fill_opacity=0.18).move_to(center)
    core = Dot(point=center, radius=radius, color=color).set_z_index(3)
    group = VGroup(halo, core)
    if label:
        tag = Text(label, font_size=20, color=WHITE).next_to(core, UP, buff=0.12)
        group.add(tag)
    return group


def build_path_mobject(points: tuple[Point2D, ...], mapper: CoordinateMapper, color: str) -> VMobject:
    path = VMobject()
    path.set_points_as_corners([mapper(pt) for pt in points])
    path.set_stroke(color=color, width=4, opacity=0.95)
    path.set_z_index(1)
    return path


def build_visibility_mobject(
    source_point: Point2D,
    polygon: tuple[Point2D, ...],
    mapper: CoordinateMapper,
    color: str,
) -> VMobject:
    visibility_points = visibility_polygon_from_point(source_point, polygon)

    if len(visibility_points) < 3:
        return VGroup()

    visibility = Polygon(
        *[mapper(pt) for pt in visibility_points],
        color=color,
        stroke_width=1.5,
        fill_color=color,
        fill_opacity=0.14,
    )
    visibility.joint_type = LineJointType.ROUND
    visibility.set_z_index(0.5)
    return visibility


class GuardPatrolScene(MovingCameraScene):
    def construct(self) -> None:
        if ACTIVE_SCENE_SPEC is None:
            raise RuntimeError("No scene specification loaded.")
        spec = ACTIVE_SCENE_SPEC

        points_for_frame = list(spec.polygon)
        for guard in spec.guards:
            if guard.point is not None:
                points_for_frame.append(guard.point)
            else:
                points_for_frame.extend(list(guard.path))
        mapper = CoordinateMapper(points_for_frame)

        polygon_mobject = Polygon(
            *[mapper(pt) for pt in spec.polygon],
            color=spec.polygon_stroke,
            stroke_width=4,
            fill_color=spec.polygon_fill,
            fill_opacity=0.92,
        )
        polygon_mobject.joint_type = LineJointType.ROUND
        polygon_mobject.set_z_index(0)

        title = None
        if spec.title:
            title = Text(spec.title, font_size=28, color="#dbe4f0", weight=MEDIUM)
            title.to_edge(UP, buff=0.35)

        target_width = max(polygon_mobject.width * 1.12, polygon_mobject.height * 1.12 * config.frame_width / config.frame_height)
        self.camera.frame.move_to(polygon_mobject).set(width=target_width * 1.08)

        intro_outline = polygon_mobject.copy().set_fill(opacity=0)
        intro_outline.set_z_index(2)

        self.add(polygon_mobject)
        if title is not None:
            title.set_z_index(4)
            title.set_opacity(0)
            self.add(title)

        self.play(
            FadeIn(polygon_mobject, scale=0.985),
            Create(intro_outline),
            self.camera.frame.animate.set(width=target_width),
            *( [FadeIn(title, shift=0.12 * DOWN)] if title is not None else [] ),
            run_time=1.8,
            rate_func=smooth,
        )

        static_guard_visuals: list[Mobject] = []
        moving_guard_visuals: list[Mobject] = []
        moving_track_visuals: list[Mobject] = []
        visibility_visuals: list[Mobject] = []
        tracker_animations: list[Animation] = []

        for guard in spec.guards:
            if guard.point is not None:
                guard_center = guard.point
                guard_visual = make_guard_visual(mapper(guard.point), guard.color, guard.radius, guard.label)
                static_guard_visuals.append(guard_visual)
                visibility_visuals.append(
                    always_redraw(
                        lambda guard_point=guard_center, guard_color=guard.color: build_visibility_mobject(
                            guard_point,
                            spec.polygon,
                            mapper,
                            guard_color,
                        )
                    )
                )
                continue

            track = build_path_mobject(guard.path, mapper, guard.color)
            track.set_stroke(color=guard.color, width=3, opacity=0.34)
            moving_track_visuals.append(track)

            tracker = ValueTracker(0.0)
            motion_path = build_path_mobject(guard.path, mapper, guard.color)
            motion_path.set_stroke(opacity=0)

            def guard_visual_factory(
                tracker_obj: ValueTracker = tracker,
                path_obj: VMobject = motion_path,
                guard_spec: GuardSpec = guard,
            ) -> VGroup:
                progress = motion_progress(tracker_obj.get_value(), guard_spec.ping_pong)
                center = path_obj.point_from_proportion(progress)
                return make_guard_visual(center, guard_spec.color, guard_spec.radius, guard_spec.label)

            guard_visual = always_redraw(guard_visual_factory)
            guard_visual.set_z_index(3)
            moving_guard_visuals.append(guard_visual)

            def moving_guard_point(
                tracker_obj: ValueTracker = tracker,
                path_points: tuple[Point2D, ...] = guard.path,
                guard_spec: GuardSpec = guard,
            ) -> Point2D:
                progress = motion_progress(tracker_obj.get_value(), guard_spec.ping_pong)
                segment_count = len(path_points) - 1
                scaled = min(progress * segment_count, segment_count - 1e-9)
                idx = int(math.floor(scaled))
                local_t = scaled - idx
                start = path_points[idx]
                end = path_points[idx + 1]
                return (
                    start[0] + local_t * (end[0] - start[0]),
                    start[1] + local_t * (end[1] - start[1]),
                )

            visibility_visuals.append(
                always_redraw(
                    lambda point_getter=moving_guard_point, guard_color=guard.color: build_visibility_mobject(
                        point_getter(),
                        spec.polygon,
                        mapper,
                        guard_color,
                    )
                )
            )

            if guard.trail:
                trail = TracedPath(
                    guard_visual.get_center,
                    stroke_color=guard.color,
                    stroke_width=3,
                    dissipating_time=1.4,
                )
                trail.set_z_index(2)
                moving_guard_visuals.append(trail)

            tracker_animations.append(tracker.animate.set_value(guard.cycles))

        if moving_track_visuals or static_guard_visuals or moving_guard_visuals:
            self.play(
                LaggedStart(
                    *[FadeIn(track, shift=0.06 * UP) for track in moving_track_visuals],
                    *[FadeIn(guard, scale=0.85) for guard in static_guard_visuals],
                    lag_ratio=0.08,
                ),
                run_time=1.2,
            )

        if visibility_visuals:
            self.add(*visibility_visuals)
        if moving_guard_visuals:
            self.add(*moving_guard_visuals)

        if moving_track_visuals:
            self.add(*moving_track_visuals)
        if static_guard_visuals:
            self.add(*static_guard_visuals)

        if tracker_animations:
            self.play(*tracker_animations, run_time=spec.duration, rate_func=linear)

        self.wait(spec.hold_time)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render moving/static watchman guards in Manim.")
    parser.add_argument("scene_file", type=Path, help="JSON scene specification file.")
    parser.add_argument("--quality", default=None, help="Manim quality preset, e.g. low_quality or medium_quality.")
    parser.add_argument("--fps", type=int, default=None, help="Override frame rate.")
    parser.add_argument("--output-file", default=None, help="Base output filename without extension.")
    parser.add_argument("--media-dir", default=None, help="Directory for rendered Manim output.")
    parser.add_argument("--preview", action="store_true", help="Preview the rendered animation after export.")
    return parser.parse_args()


def main() -> None:
    global ACTIVE_SCENE_SPEC

    args = parse_args()
    scene_path = args.scene_file.resolve()
    ACTIVE_SCENE_SPEC = load_scene_spec(scene_path, args)

    render_config = {
        "quality": ACTIVE_SCENE_SPEC.quality,
        "frame_rate": ACTIVE_SCENE_SPEC.fps,
        "preview": ACTIVE_SCENE_SPEC.preview,
        "media_dir": ACTIVE_SCENE_SPEC.media_dir,
        "output_file": ACTIVE_SCENE_SPEC.output_file,
        "background_color": ACTIVE_SCENE_SPEC.background_color,
        "disable_caching": True,
    }

    with tempconfig(render_config):
        scene = GuardPatrolScene()
        scene.render()
        output_path = getattr(scene.renderer.file_writer, "movie_file_path", None)

    if output_path:
        print(Path(output_path).resolve())


if __name__ == "__main__":
    main()
