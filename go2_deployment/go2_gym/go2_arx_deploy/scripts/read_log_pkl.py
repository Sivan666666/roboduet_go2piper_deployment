#!/usr/bin/env python3
import argparse
import math
import pickle
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


CMDS_LABELS = [
    "v_x",
    "v_y",
    "yaw_rate",
]
EE_CMD_LABELS = ["x", "y", "z", "roll", "pitch", "yaw"]
ARM_ACTION_LABELS = [f"arm_action_{idx}" for idx in range(6)]
ARM_JOINT_LABELS = [f"piper_joint{idx}" for idx in range(1, 7)]
DOG_ACTION_LABELS = [
    "dog_action_0", "dog_action_1", "dog_action_2",
    "dog_action_3", "dog_action_4", "dog_action_5",
    "dog_action_6", "dog_action_7", "dog_action_8",
    "dog_action_9", "dog_action_10", "dog_action_11",
]
DOG_JOINT_LABELS = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]
DOG_LEG_SLICE = slice(0, 3)
DOG_LEG_ACTION_LABELS = ["FL_action_hip", "FL_action_thigh", "FL_action_calf"]
DOG_LEG_JOINT_LABELS = ["FL_hip", "FL_thigh", "FL_calf"]
COLORS = [
    (37, 99, 235),
    (220, 38, 38),
    (5, 150, 105),
    (202, 138, 4),
    (168, 85, 247),
    (234, 88, 12),
    (14, 165, 233),
    (236, 72, 153),
    (34, 197, 94),
    (100, 116, 139),
    (245, 158, 11),
    (124, 58, 237),
]
ACTUAL_COLOR = (29, 78, 216)
ACTION_COLOR = (234, 88, 12)


def load_log(path):
    with open(path, "rb") as file:
        return pickle.load(file)


def to_series(infos, key):
    values = []
    for info in infos:
        if key not in info:
            continue
        values.append(np.asarray(info[key]).squeeze())
    if not values:
        raise KeyError(f"Key '{key}' not found in log infos")
    return np.asarray(values)


def get_time_axis(infos):
    times = [info.get("time") for info in infos]
    if any(value is None for value in times):
        return np.arange(len(infos), dtype=np.float64)
    return np.asarray(times, dtype=np.float64)


def estimate_active_window(action_data, actual_data, pad_ratio=0.08, tail_ratio=0.15):
    num_steps = action_data.shape[0]
    if num_steps <= 2:
        return 0, num_steps

    tail_len = max(5, int(num_steps * tail_ratio))
    tail_action = np.median(action_data[-tail_len:], axis=0)
    tail_actual = np.median(actual_data[-tail_len:], axis=0)

    action_dev = np.max(np.abs(action_data - tail_action), axis=1)
    actual_dev = np.max(np.abs(actual_data - tail_actual), axis=1)
    score = np.maximum(action_dev, actual_dev)

    score_max = float(np.max(score))
    if score_max <= 1e-9:
        return 0, num_steps

    threshold = max(score_max * 0.08, float(np.percentile(score, 70)) * 0.5)
    active_idx = np.flatnonzero(score > threshold)
    if active_idx.size == 0:
        return 0, num_steps

    start = int(active_idx[0])
    end = int(active_idx[-1]) + 1
    pad = max(3, int((end - start) * pad_ratio))
    start = max(0, start - pad)
    end = min(num_steps, end + pad)
    return start, end


def lpy_to_local_xyz(lpy):
    l, p, yaw = lpy
    x = l * np.cos(p) * np.cos(yaw)
    y_local = l * np.cos(p) * np.sin(yaw)
    z = l * np.sin(p)
    return np.array([x, y_local, z], dtype=np.float64)


def cmds_to_ee_xyzrpy(cmds):
    ee_xyz = np.asarray([lpy_to_local_xyz(row[3:6]) for row in cmds], dtype=np.float64)
    ee_rpy = np.asarray(cmds[:, 6:9], dtype=np.float64)
    return np.concatenate((ee_xyz, ee_rpy), axis=1)


def pad_range(vmin, vmax):
    if not np.isfinite(vmin) or not np.isfinite(vmax):
        return -1.0, 1.0
    if abs(vmax - vmin) < 1e-9:
        pad = 1.0 if abs(vmin) < 1e-9 else abs(vmin) * 0.1
        return vmin - pad, vmax + pad
    pad = 0.08 * (vmax - vmin)
    return vmin - pad, vmax + pad


def make_canvas(width, height):
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    return image, draw, font


def draw_text(draw, pos, text, font, fill=(0, 0, 0)):
    draw.text(pos, text, font=font, fill=fill)


def draw_plot_box(draw, box):
    left, top, right, bottom = box
    draw.rectangle(box, outline=(160, 160, 160), width=1)
    for frac in (0.25, 0.5, 0.75):
        y = top + (bottom - top) * frac
        draw.line((left, y, right, y), fill=(235, 235, 235), width=1)


def map_points(x_values, y_values, box, y_min, y_max):
    left, top, right, bottom = box
    width = max(1, right - left)
    height = max(1, bottom - top)
    x0 = float(x_values[0])
    x1 = float(x_values[-1]) if len(x_values) > 1 else float(x_values[0]) + 1.0
    if abs(x1 - x0) < 1e-9:
        x1 = x0 + 1.0

    points = []
    for x_val, y_val in zip(x_values, y_values):
        x = left + width * float((x_val - x0) / (x1 - x0))
        y = bottom - height * float((y_val - y_min) / (y_max - y_min))
        points.append((x, y))
    return points


def draw_single_series_panel(draw, font, time_axis, series, box, title):
    draw_plot_box(draw, box)
    y_min, y_max = pad_range(float(np.min(series)), float(np.max(series)))
    points = map_points(time_axis, series, box, y_min, y_max)
    if len(points) >= 2:
        draw.line(points, fill=COLORS[0], width=2)

    left, top, right, bottom = box
    draw_text(draw, (left, top - 14), title, font)
    draw_text(draw, (left + 4, top + 2), f"max={y_max:.3f}", font, fill=(90, 90, 90))
    draw_text(draw, (left + 4, bottom - 12), f"min={y_min:.3f}", font, fill=(90, 90, 90))


def draw_dashed_polyline(draw, points, color, width=2, dash_px=8, gap_px=5):
    if len(points) < 2:
        return

    for idx in range(len(points) - 1):
        x0, y0 = points[idx]
        x1, y1 = points[idx + 1]
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len <= 1e-9:
            continue

        step = dash_px + gap_px
        traveled = 0.0
        while traveled < seg_len:
            dash_end = min(traveled + dash_px, seg_len)
            start_ratio = traveled / seg_len
            end_ratio = dash_end / seg_len
            sx = x0 + dx * start_ratio
            sy = y0 + dy * start_ratio
            ex = x0 + dx * end_ratio
            ey = y0 + dy * end_ratio
            draw.line((sx, sy, ex, ey), fill=color, width=width)
            traveled += step


def draw_joint_overlay_panel(draw, font, time_axis, action_series, actual_series, box, title):
    draw_plot_box(draw, box)
    y_min, y_max = pad_range(
        float(min(np.min(action_series), np.min(actual_series))),
        float(max(np.max(action_series), np.max(actual_series))),
    )
    left, top, right, bottom = box
    draw_text(draw, (left, top - 14), title, font)
    draw_text(draw, (left + 4, top + 2), f"max={y_max:.3f}", font, fill=(90, 90, 90))
    draw_text(draw, (left + 4, bottom - 12), f"min={y_min:.3f}", font, fill=(90, 90, 90))

    legend_y = top + 18
    draw.line((left + 6, legend_y + 4, left + 22, legend_y + 4), fill=ACTUAL_COLOR, width=2)
    draw_text(draw, (left + 26, legend_y), "actual", font, fill=ACTUAL_COLOR)
    draw_dashed_polyline(draw, [(left + 92, legend_y + 4), (left + 108, legend_y + 4)], color=ACTION_COLOR, width=2)
    draw_text(draw, (left + 114, legend_y), "action", font, fill=ACTION_COLOR)

    actual_points = map_points(time_axis, actual_series, box, y_min, y_max)
    action_points = map_points(time_axis, action_series, box, y_min, y_max)
    if len(actual_points) >= 2:
        draw.line(actual_points, fill=ACTUAL_COLOR, width=2)
    if len(action_points) >= 2:
        draw_dashed_polyline(draw, action_points, color=ACTION_COLOR, width=2)


def create_grid_figure(time_axis, data, labels, title, cols=3, panel_width=420, panel_height=220):
    rows = int(math.ceil(len(labels) / cols))
    margin_x = 28
    margin_y = 36
    gap_x = 22
    gap_y = 34
    width = margin_x * 2 + cols * panel_width + (cols - 1) * gap_x
    height = 50 + margin_y * 2 + rows * panel_height + (rows - 1) * gap_y
    image, draw, font = make_canvas(width, height)
    draw_text(draw, (margin_x, 16), title, font)

    for idx, label in enumerate(labels):
        row = idx // cols
        col = idx % cols
        left = margin_x + col * (panel_width + gap_x)
        top = 50 + margin_y + row * (panel_height + gap_y)
        box = (left, top, left + panel_width, top + panel_height)
        draw_single_series_panel(draw, font, time_axis, data[:, idx], box, label)

    return image


def create_joint_tracking_figure(time_axis, action_data, actual_data, labels, title, cols=3, panel_width=420, panel_height=220):
    rows = int(math.ceil(len(labels) / cols))
    margin_x = 28
    margin_y = 36
    gap_x = 22
    gap_y = 34
    width = margin_x * 2 + cols * panel_width + (cols - 1) * gap_x
    height = 74 + margin_y * 2 + rows * panel_height + (rows - 1) * gap_y
    image, draw, font = make_canvas(width, height)
    draw_text(draw, (margin_x, 16), title, font)
    legend_y = 38
    draw.line((margin_x, legend_y + 5, margin_x + 18, legend_y + 5), fill=ACTUAL_COLOR, width=2)
    draw_text(draw, (margin_x + 24, legend_y), "actual joint state", font, fill=ACTUAL_COLOR)
    draw_dashed_polyline(draw, [(margin_x + 180, legend_y + 5), (margin_x + 198, legend_y + 5)], color=ACTION_COLOR, width=2)
    draw_text(draw, (margin_x + 206, legend_y), "action", font, fill=ACTION_COLOR)

    for idx, label in enumerate(labels):
        row = idx // cols
        col = idx % cols
        left = margin_x + col * (panel_width + gap_x)
        top = 74 + margin_y + row * (panel_height + gap_y)
        box = (left, top, left + panel_width, top + panel_height)
        draw_joint_overlay_panel(
            draw,
            font,
            time_axis,
            action_data[:, idx],
            actual_data[:, idx],
            box,
            label,
        )
    return image


def crop_tracking_data(time_axis, action_data, actual_data):
    start, end = estimate_active_window(action_data, actual_data)
    return time_axis[start:end], action_data[start:end], actual_data[start:end]


def save_image(image, path):
    image.save(path)
    print(f"saved: {path}")


def parse_args():
    parser = argparse.ArgumentParser(description="Read deployment log.pkl and draw key curves")
    parser.add_argument("log_pkl", help="Path to log.pkl")
    parser.add_argument("--robot", default="", help="Robot name inside log.pkl; default uses the first one")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Directory to save generated figures; default uses the log.pkl parent directory",
    )
    parser.add_argument("--show", action="store_true", help="Open generated images with the default system viewer")
    return parser.parse_args()


def main():
    args = parse_args()
    log_path = Path(args.log_pkl).resolve()
    if not log_path.exists():
        raise FileNotFoundError(f"log file not found: {log_path}")
    if log_path.stat().st_size == 0:
        raise ValueError(f"log file is empty: {log_path}")

    log_data = load_log(log_path)
    if not log_data:
        raise ValueError(f"log file contains no robot entries: {log_path}")

    robot_name = args.robot or next(iter(log_data))
    if robot_name not in log_data:
        raise KeyError(f"Robot '{robot_name}' not found. Available: {list(log_data.keys())}")

    _, infos = log_data[robot_name]
    if not infos:
        raise ValueError(f"log file contains no timestep infos for robot '{robot_name}'")

    time_axis = get_time_axis(infos)
    cmds = to_series(infos, "cmds_vxyz_lpyrpy")
    arm_action = to_series(infos, "arm_action")
    dog_action = to_series(infos, "dog_action")
    joint_pos = to_series(infos, "joint_pos")

    arm_action = arm_action[:, :6]
    arm_joint_state = joint_pos[:, 12:18]
    dog_joint_state = joint_pos[:, :12]
    dog_action = dog_action[:, DOG_LEG_SLICE]
    dog_joint_state = dog_joint_state[:, DOG_LEG_SLICE]

    output_dir = Path(args.output_dir).resolve() if args.output_dir else log_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    base_cmd_image = create_grid_figure(
        time_axis,
        cmds[:, :3],
        CMDS_LABELS,
        title=f"{robot_name} | base command",
        cols=3,
    )
    ee_cmds = cmds_to_ee_xyzrpy(cmds)
    ee_cmd_image = create_grid_figure(
        time_axis,
        ee_cmds,
        EE_CMD_LABELS,
        title=f"{robot_name} | end effector command xyzrpy",
        cols=3,
    )
    arm_time_axis, arm_action_plot, arm_joint_state_plot = crop_tracking_data(time_axis, arm_action, arm_joint_state)
    arm_image = create_joint_tracking_figure(
        arm_time_axis,
        arm_action_plot,
        arm_joint_state_plot,
        ARM_JOINT_LABELS,
        title=f"{robot_name} | arm action vs actual arm joint state",
        cols=1,
    )
    dog_time_axis, dog_action_plot, dog_joint_state_plot = crop_tracking_data(time_axis, dog_action, dog_joint_state)
    dog_image = create_joint_tracking_figure(
        dog_time_axis,
        dog_action_plot,
        dog_joint_state_plot,
        DOG_LEG_JOINT_LABELS,
        title=f"{robot_name} | FL leg action vs actual joint state",
        cols=1,
    )

    output_paths = [
        output_dir / "cmds_base_command.png",
        output_dir / "cmds_end_effector_xyzrpy.png",
        output_dir / "arm_action_vs_arm_joint_state.png",
        output_dir / "dog_action_vs_dog_joint_state.png",
    ]
    for image, path in zip((base_cmd_image, ee_cmd_image, arm_image, dog_image), output_paths):
        save_image(image, path)

    if args.show:
        for path in output_paths:
            Image.open(path).show()


if __name__ == "__main__":
    main()
