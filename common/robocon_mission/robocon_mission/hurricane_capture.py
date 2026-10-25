"""Capture hurricane field geometry in ROS ``map``; offline tools need no ROS.

``start`` and ``finish`` reproduce hurricane's two-point field-axis calibration.
Here ``finish`` means the second axis marker, not a navigation goal.  Each
obstacle needs ``<skill>.begin``, ``<skill>.entry`` and ``<skill>.exit``.  All
recorded headings are ROS yaw in radians.  Source hurricane positions must
never be copied into these map samples without a surveyed transformation.
"""

import argparse
from copy import deepcopy
from math import atan2, cos, hypot, isfinite, pi, radians, sin
import os
from pathlib import Path
import sys
import tempfile

import yaml


SKILL_IDS = (
    "bridge_a", "bridge_b", "big_slope", "limit_pole", "bunker", "pole",
    "high_wall", "step",
)
SKILL_NAMES = (
    "木桥A", "木桥B", "大斜坡", "限高栏", "沙坑", "绕杆", "高墙", "台阶",
)
LABELS = ("start", "finish") + tuple(
    f"{skill}.{part}" for skill in SKILL_IDS for part in ("begin", "entry", "exit")
)


def selected_skills(skills=None):
    """Selection filters the established course order; it does not reorder it."""
    if skills is None:
        return SKILL_IDS
    if not skills or any(skill not in SKILL_IDS for skill in skills):
        raise ValueError('select at least one known obstacle')
    if len(set(skills)) != len(skills):
        raise ValueError('duplicate selected obstacle')
    return tuple(skill for skill in SKILL_IDS if skill in skills)


def required_labels(skills=None):
    return ('start', 'finish') + tuple(
        f'{skill}.{part}' for skill in selected_skills(skills)
        for part in ('begin', 'entry', 'exit'))


def _pose(value, label):
    if isinstance(value, dict):
        if any(key not in value for key in ("x", "y", "yaw")):
            raise ValueError(f"{label} requires x, y and yaw")
        value = [value[key] for key in ("x", "y", "yaw")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} must be [map x, map y, yaw radians]")
    if any(isinstance(item, bool) or not isinstance(item, (int, float))
           or not isfinite(item) for item in value):
        raise ValueError(f"{label} must contain three finite numbers")
    return tuple(float(item) for item in value)


def _read_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(f"invalid YAML in {path}: {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"{path}: YAML root must be a mapping")
    return data


def _write_yaml(path, data, comment):
    """Replace one output atomically so interrupted saves retain old samples."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=path.parent,
                prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = stream.name
            stream.write(comment.rstrip() + "\n")
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        os.replace(temporary, path)
    finally:
        if temporary is not None and Path(temporary).exists():
            Path(temporary).unlink()
    return path


class FieldCalibration:
    """Map sample store; exports require complete geometry for selected skills."""

    def __init__(self, samples=None, metadata=None):
        self.samples = {}
        self.metadata = deepcopy(metadata or {})
        if samples is not None:
            if not isinstance(samples, dict):
                raise ValueError("samples must be a mapping")
            for label, pose in samples.items():
                self.record(label, pose)

    def record(self, label, pose):
        if label not in LABELS:
            raise ValueError(f"unknown capture label: {label}")
        self.samples[label] = _pose(pose, label)
        return self.samples[label]

    @property
    def missing(self):
        return tuple(label for label in LABELS if label not in self.samples)

    @property
    def axis_yaw(self):
        if "start" not in self.samples or "finish" not in self.samples:
            raise ValueError("start and finish axis samples are required")
        start, finish = self.samples["start"], self.samples["finish"]
        dx, dy = finish[0] - start[0], finish[1] - start[1]
        if hypot(dx, dy) <= 1e-6:
            raise ValueError("start and finish axis markers must be distinct")
        return atan2(dy, dx)

    @property
    def calibrated(self):
        if self.missing:
            return False
        try:
            self.axis_yaw
        except ValueError:
            return False
        return True

    def _require_complete(self, selected=None):
        missing = [label for label in required_labels(selected) if label not in self.samples]
        if missing:
            raise ValueError("missing map samples: " + ", ".join(missing))
        # Revalidate in case a caller modified the public samples dictionary.
        for label, pose in self.samples.items():
            if label not in LABELS:
                raise ValueError(f"unknown capture label: {label}")
            _pose(pose, label)
        return self.axis_yaw

    def save(self, path):
        validated = {}
        for label, pose in self.samples.items():
            if label not in LABELS:
                raise ValueError(f"unknown capture label: {label}")
            validated[label] = _pose(pose, label)
        data = deepcopy(self.metadata)
        data.update({
            "version": 1,
            "frame_id": "map",
            "pose_units": {"position": "metres", "yaw": "radians"},
            "calibrated": self.calibrated,
            "samples": {label: dict(zip(("x", "y", "yaw"), pose))
                        for label, pose in validated.items()},
        })
        return _write_yaml(path, data,
                           "# Captured ROS map poses; finish is the field-axis marker.")

    @classmethod
    def load(cls, path):
        data = _read_yaml(path)
        if data.get("version") != 1 or isinstance(data.get("version"), bool):
            raise ValueError("field calibration version must be 1")
        if data.get("frame_id") != "map":
            raise ValueError("field calibration frame_id must be map")
        samples = data.get("samples")
        if not isinstance(samples, dict):
            raise ValueError("field calibration samples must be a mapping")
        units = data.get("pose_units")
        if units is not None and units != {"position": "metres", "yaw": "radians"}:
            raise ValueError("map positions must be metres and yaw must be radians")
        result = cls(samples, {key: value for key, value in data.items()
                               if key not in ("version", "frame_id", "pose_units",
                                              "samples", "calibrated")})
        if data.get("calibrated") is True and not result.calibrated:
            raise ValueError("calibrated field file has incomplete/invalid samples")
        return result

    def export_route(self, path, selected=None):
        selected = selected_skills(selected)
        self._require_complete(selected)
        tasks = [{
            "id": skill,
            "type": "obstacle",
            "transit_pose": list(self.samples[f"{skill}.begin"]),
            "approach_pose": list(self.samples[f"{skill}.entry"]),
            "end_pose": list(self.samples[f"{skill}.exit"]),
            "skill_id": skill,
            "timeout": 180.0,
            "max_retries": 0,
        } for skill in selected]
        return _write_yaml(path, {
            "start_area": "hurricane_calibrated_start",
            "frame_id": "map",
            "route": tasks,
            "return": {"pose": list(self.samples["start"])},
        }, "# Site-captured map route. Axis end is not a navigation goal.")

    def export_skills(self, profile_path, out_path, selected=None):
        selected = selected_skills(selected)
        axis_yaw = self._require_complete(selected)
        profile = _read_yaml(profile_path)
        skills = profile.get("skills")
        if not isinstance(skills, dict) or any(skill not in skills for skill in selected):
            scope = 'all eight' if selected == SKILL_IDS else 'all selected'
            raise ValueError(f"skill profile must contain {scope} hurricane skills")
        skills = {skill: skills[skill] for skill in selected}
        profile['skills'] = skills
        warnings = []
        for skill in selected:
            if not isinstance(skills[skill], dict):
                raise ValueError(f"skill {skill} must be a mapping")
            begin = self.samples[f"{skill}.begin"]
            # Handler axes are common to the full course, not the robot's
            # heading at begin. The execution layer reflects source y/yaw.
            skills[skill]["origin"] = [begin[0], begin[1], axis_yaw]
            skills[skill]["entry_pose"] = list(self.samples[f"{skill}.entry"])
            skills[skill]["expected_exit_pose"] = list(self.samples[f"{skill}.exit"])
            skills[skill]["calibrated"] = True
            steps = skills[skill].get("steps", [])
            if not isinstance(steps, list):
                raise ValueError(f"skill {skill}.steps must be a list")
            for index, step in enumerate(steps):
                if not isinstance(step, dict):
                    raise ValueError(f"skill {skill}.steps[{index}] must be a mapping")
                if "anchor_skill" not in step and "anchor_offset" not in step:
                    continue
                label = f"{skill}.steps[{index}].anchor"
                anchor = step.get("anchor_skill")
                if step.get("type") != "waypoint" or anchor not in SKILL_IDS:
                    raise ValueError(f"{label} requires a waypoint and a known anchor_skill")
                offset = step.get("anchor_offset")
                if not isinstance(offset, (list, tuple)) or len(offset) != 2:
                    raise ValueError(f"{label}_offset must be two finite source-axis numbers")
                offset_x, offset_y, _ = _pose([*offset, 0], label + "_offset")
                target = step.get("target")
                if not isinstance(target, (list, tuple)) or len(target) != 3:
                    raise ValueError(f"{label}.target must be [x, y, yaw degrees or null]")
                yaw = (None if target[2] is None else
                       _pose([0, 0, target[2]], label + ".yaw")[2])
                if f'{anchor}.begin' not in self.samples:
                    raise ValueError(f'{label} requires map sample {anchor}.begin; '
                                     'capture the anchor origin without selecting its obstacle')
                anchor_begin = self.samples[f"{anchor}.begin"]
                dx, dy = anchor_begin[0] - begin[0], anchor_begin[1] - begin[1]
                # Convert surveyed map displacement to hurricane's reflected
                # global field axes, then apply the original source offset.
                source_x = dx * cos(axis_yaw) + dy * sin(axis_yaw)
                source_y = dx * sin(axis_yaw) - dy * cos(axis_yaw)
                step["target"] = [source_x + offset_x, source_y + offset_y, yaw]
            # A fully specified terminal waypoint predicts an exit. Flag a
            # measured mismatch for operator review, without inventing motion
            # to the measured exit or treating it as a Nav2 obstacle crossing.
            last = steps[-1] if steps else {}
            target = last.get("target") if last.get("type") == "waypoint" else None
            if (isinstance(target, (list, tuple)) and len(target) == 3
                    and all(value is not None for value in target)):
                x, reflected_y, yaw_deg = _pose(target, f"{skill}.terminal_target")
                predicted = (
                    begin[0] + cos(axis_yaw) * x + sin(axis_yaw) * reflected_y,
                    begin[1] + sin(axis_yaw) * x - cos(axis_yaw) * reflected_y,
                    axis_yaw - radians(yaw_deg),
                )
                exit_pose = self.samples[f"{skill}.exit"]
                distance = hypot(predicted[0] - exit_pose[0], predicted[1] - exit_pose[1])
                yaw_error = abs((predicted[2] - exit_pose[2] + pi) % (2 * pi) - pi)
                if distance > 0.1 or yaw_error > radians(4):
                    warnings.append({
                        "skill_id": skill,
                        "reason": "measured exit differs from the configured terminal waypoint",
                        "position_error_m": distance,
                        "yaw_error_rad": yaw_error,
                        "configured_exit_pose": list(predicted),
                        "measured_exit_pose": list(exit_pose),
                    })
        profile["calibration"] = {
            "frame_id": "map",
            "axis_start": list(self.samples["start"]),
            "axis_end": list(self.samples["finish"]),
            "axis_yaw": axis_yaw,
            "source": "site-captured map poses",
            "warnings": warnings,
        }
        return _write_yaml(out_path, profile,
                           "# Site-calibrated origins; source waypoints retain hurricane convention.")


def _default_skills_profile():
    local = Path(__file__).resolve().parent.parent / "config" / "hurricane_skills.yaml"
    if local.exists():
        return str(local)
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("robocon_mission")) /
                   "config" / "hurricane_skills.yaml")
    except (ImportError, LookupError):
        return str(local)


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", required=True, help="map sample YAML to create or update")
    parser.add_argument("--label", choices=LABELS, help="sample label")
    parser.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                        help="manual map pose; metres and radians")
    parser.add_argument("--select", nargs='+', choices=SKILL_IDS,
                        help="export only these obstacles, in standard course order (default: all)")
    parser.add_argument("--export-route", metavar="YAML", help="export selected competition route")
    parser.add_argument("--export-skills", metavar="YAML", help="export selected calibrated skills")
    parser.add_argument("--skills-profile", default=_default_skills_profile(),
                        help="source hurricane_skills.yaml for skill export")
    parser.add_argument("--panel", action="store_true", help="show ROS TF capture panel")
    parser.add_argument("--base-frame", default="base_footprint", help="robot TF frame")
    parser.add_argument("--tf-max-age", type=float, default=0.5,
                        help="oldest accepted dynamic map TF in seconds")
    return parser


def _run_panel(calibration, options, ros_args):
    # ROS and Tk imports are intentionally lazy for offline export and --help.
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener, TransformException
        import tkinter as tk
        from tkinter import messagebox
    except ImportError as error:
        raise RuntimeError("capture panel requires ROS 2, tf2_ros and tkinter") from error

    rclpy.init(args=ros_args)
    node = Node("hurricane_capture_panel")
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # Keep listener alive for the panel.
    root = None
    try:
        try:
            root = tk.Tk()
        except tk.TclError as error:
            raise RuntimeError("cannot open Tk panel; use a graphical ROS session") from error
        root.title("Hurricane 现场采点 · map")
        status = tk.StringVar(value="等待 map → " + options.base_frame + " TF")
        progress = tk.StringVar()
        selections = {skill: tk.BooleanVar(value=skill in selected_skills(options.select))
                      for skill in SKILL_IDS}

        def selection():
            return [skill for skill in SKILL_IDS if selections[skill].get()]
        tk.Label(root, text="起点与轴终点定义场地 +X 轴。各点 yaw 为 ROS map 弧度。",
                 wraplength=660).grid(row=0, column=0, columnspan=4, padx=10, pady=8)
        tk.Label(root, textvariable=status, wraplength=660).grid(
            row=1, column=0, columnspan=4, padx=10, pady=6)
        tk.Label(root, textvariable=progress).grid(row=2, column=0, columnspan=4)

        def current_pose():
            transform = buffer.lookup_transform("map", options.base_frame, Time())
            stamp = (transform.header.stamp.sec * 1_000_000_000 +
                     transform.header.stamp.nanosec)
            age = (node.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp == 0 or age < -0.1 or age > options.tf_max_age:
                raise ValueError(f"TF is stale/future (age={age:.3f}s); capture refused")
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = [rotation.x, rotation.y, rotation.z, rotation.w]
            if any(not isfinite(value) for value in values):
                raise ValueError("TF quaternion is not finite")
            norm = sum(value * value for value in values) ** 0.5
            if norm <= 1e-12:
                raise ValueError("TF quaternion is invalid")
            x, y, z, w = [value / norm for value in values]
            yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            return _pose([translation.x, translation.y, yaw], "map TF")

        def update_progress():
            if not selection():
                progress.set('请至少选择一个障碍')
                return
            labels = required_labels(selection())
            count = sum(label in calibration.samples for label in labels)
            progress.set(f"所选障碍基础点 {count}/{len(labels)}；技能锚点依赖在导出时检查")

        def capture(label):
            try:
                pose = calibration.record(label, current_pose())
                calibration.save(options.file)
                status.set(f"已保存 {label}: x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={pose[2]:.3f}")
                update_progress()
            except (TransformException, ValueError, OSError) as error:
                messagebox.showerror("采点失败", str(error), parent=root)

        tk.Button(root, text="采集起点 start", command=lambda: capture("start")).grid(
            row=3, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        tk.Button(root, text="采集场地轴终点 finish", command=lambda: capture("finish")).grid(
            row=3, column=2, columnspan=2, sticky="ew", padx=4, pady=4)
        for index, (skill, name) in enumerate(zip(SKILL_IDS, SKILL_NAMES), start=4):
            tk.Checkbutton(root, text=name, variable=selections[skill],
                           command=update_progress).grid(row=index, column=0, sticky="w", padx=8)
            for column, (part, name_part) in enumerate(
                    (("begin", "原点 begin"), ("entry", "入口 entry"),
                     ("exit", "出口 exit")), start=1):
                label = f"{skill}.{part}"
                tk.Button(root, text=name_part,
                          command=lambda label=label: capture(label)).grid(
                    row=index, column=column, sticky="ew", padx=4, pady=3)

        def output(action):
            try:
                if action == "save":
                    path = calibration.save(options.file)
                elif action == "route":
                    path = calibration.export_route(options.export_route or
                        str(Path(options.file).with_name("hurricane_route_calibrated.yaml")), selection())
                else:
                    path = calibration.export_skills(options.skills_profile,
                        options.export_skills or
                        str(Path(options.file).with_name("hurricane_skills_calibrated.yaml")), selection())
                status.set("已保存：" + str(path))
            except (ValueError, OSError) as error:
                messagebox.showerror("导出失败", str(error), parent=root)

        for column, (text, action) in enumerate(
                (("保存采点", "save"), ("导出所选路线", "route"),
                 ("导出所选技能", "skills"))):
            tk.Button(root, text=text, command=lambda action=action: output(action)).grid(
                row=12, column=column, sticky="ew", padx=4, pady=8)
        update_progress()

        def poll():
            if not rclpy.ok():
                root.destroy()
                return
            rclpy.spin_once(node, timeout_sec=0.0)
            try:
                pose = current_pose()
                status.set(f"map TF：x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={pose[2]:.3f}")
            except (TransformException, ValueError) as error:
                status.set("不可采点：" + str(error))
            root.after(50, poll)

        root.after(50, poll)
        root.mainloop()
    finally:
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass
        del listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    parser = _parser()
    options, ros_args = parser.parse_known_args(args)
    if ros_args and not options.panel:
        parser.error("unrecognized arguments: " + " ".join(ros_args))
    if bool(options.label) != bool(options.pose):
        parser.error("--label and --pose must be supplied together for manual capture")
    if (not isfinite(options.tf_max_age) or options.tf_max_age <= 0):
        parser.error("--tf-max-age must be finite and positive")
    if options.base_frame.strip() in ("", "map"):
        parser.error("--base-frame must be a robot frame different from map")
    try:
        selected = selected_skills(options.select)
        calibration = (FieldCalibration.load(options.file) if Path(options.file).exists()
                       else FieldCalibration())
        if options.pose:
            calibration.record(options.label, options.pose)
            calibration.save(options.file)
            print("Captured " + options.label + " in " + options.file)
        if options.panel:
            _run_panel(calibration, options, ros_args)
            return
        if options.export_route:
            calibration.export_route(options.export_route, selected)
            print("Exported route: " + options.export_route)
        if options.export_skills:
            calibration.export_skills(options.skills_profile, options.export_skills, selected)
            print("Exported calibrated skills: " + options.export_skills)
        if not any((options.pose, options.export_route, options.export_skills)):
            print("Missing map samples: " + ", ".join(
                label for label in required_labels(selected) if label not in calibration.samples))
    except (ValueError, OSError, RuntimeError) as error:
        parser.error(str(error))


def panel_main(args=None):
    argv = list(sys.argv[1:] if args is None else args)
    if "--panel" not in argv:
        argv.insert(0, "--panel")
    main(argv)


if __name__ == "__main__":
    main()
