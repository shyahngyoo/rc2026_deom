"""Strict YAML route parsing; all poses are [x, y, yaw] in map."""

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Optional, Tuple

import yaml


Pose = Tuple[float, float, float]


class StrictLoader(yaml.SafeLoader):
    """Reject duplicate mapping keys rather than silently overriding safety limits."""

    def construct_mapping(self, node, deep=False):
        keys = [self.construct_object(key, deep=deep) for key, _ in node.value]
        if any(not isinstance(key, str) for key in keys) or len(keys) != len(set(keys)):
            raise ValueError("YAML mapping keys must be unique strings")
        return super().construct_mapping(node, deep=deep)


def _unknown(mapping, allowed, label):
    unknown = set(mapping) - allowed
    if unknown:
        raise ValueError(f"{label}: unknown fields {sorted(map(str, unknown))}")


def _pose(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} must be [x, y, yaw]")
    if any(isinstance(item, bool) or not isinstance(item, (int, float))
           or not isfinite(item) for item in value):
        raise ValueError(f"{label} must contain three finite numbers")
    return tuple(float(item) for item in value)


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return float(value)


def _retries(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a nonnegative integer")
    return value


@dataclass(frozen=True)
class Task:
    id: str
    type: str
    pose: Optional[Pose] = None
    transit_pose: Optional[Pose] = None
    approach_pose: Optional[Pose] = None
    skill_id: str = ""
    timeout: Optional[float] = None
    max_retries: Optional[int] = None
    end_pose: Optional[Pose] = None
    end_xy_tolerance: float = 0.15
    end_yaw_tolerance: float = 0.20
    verify_timeout: float = 5.0
    transit_timeout: Optional[float] = None
    approach_timeout: Optional[float] = None
    skill_timeout: Optional[float] = None
#把yaml转换成task

class RouteManager:
    def __init__(self, start_area, frame_id, tasks, return_pose):
        self.start_area = start_area
        self.frame_id = frame_id
        self.tasks = tuple(tasks)
        self.return_pose = return_pose
        self.index = 0
        self.completed = []

    @classmethod
    def load(cls, path):
        with Path(path).open(encoding="utf-8") as stream:
            try:
                data = yaml.load(stream, Loader=StrictLoader)
            except yaml.YAMLError as error:
                raise ValueError(f"invalid route YAML: {error}") from error
        if not isinstance(data, dict):
            raise ValueError("route root must be a mapping")
        _unknown(data, {"start_area", "frame_id", "route", "return"}, "route root")
        if not isinstance(data.get("start_area"), str) or not data["start_area"].strip():
            raise ValueError("start_area is required")
        if data.get("frame_id") != "map":
            raise ValueError("frame_id must be map")
        entries = data.get("route")
        if not isinstance(entries, list) or not entries:
            raise ValueError("route must be a nonempty list")
        ret = data.get("return")
        if not isinstance(ret, dict):
            raise ValueError("return.pose is required")
        _unknown(ret, {"pose"}, "return")
        return_pose = _pose(ret.get("pose"), "return.pose")
        ids = set()
        tasks = []
        for index, entry in enumerate(entries):
            label = f"route[{index}]"
            if not isinstance(entry, dict):
                raise ValueError(f"{label} must be a mapping")
            task_id = entry.get("id")
            if not isinstance(task_id, str) or not task_id.strip() or task_id in ids:
                raise ValueError(f"{label}.id missing or duplicated")
            ids.add(task_id)
            kind = entry.get("type")
            if kind not in ("navigation", "obstacle"):
                raise ValueError(f"{label}.type invalid")
            common = {"id", "type", "timeout", "max_retries"}
            fields = ({"pose", "transit_timeout"} if kind == "navigation" else {
                "transit_pose", "approach_pose", "skill_id", "end_pose",
                "end_xy_tolerance", "end_yaw_tolerance", "verify_timeout",
                "transit_timeout", "approach_timeout", "skill_timeout"})
            _unknown(entry, common | fields, label)
            limits = {name: _positive(entry[name], f"{label}.{name}")
                      for name in fields if name in entry and name in {
                          "end_xy_tolerance", "end_yaw_tolerance", "verify_timeout",
                          "transit_timeout", "approach_timeout", "skill_timeout"}}
            if limits.get("end_yaw_tolerance", 0.20) > 3.141592653589793:
                raise ValueError(f"{label}.end_yaw_tolerance must be <= pi")
            timeout = (None if "timeout" not in entry else
                       _positive(entry["timeout"], f"{label}.timeout"))
            retries = (None if "max_retries" not in entry else
                       _retries(entry["max_retries"], f"{label}.max_retries"))
            if kind == "navigation":
                tasks.append(Task(task_id, kind, pose=_pose(entry.get("pose"),
                                                             f"{label}.pose"),
                                  timeout=timeout, max_retries=retries, **limits))
            else:
                skill_id = entry.get("skill_id")
                if not isinstance(skill_id, str) or not skill_id.strip():
                    raise ValueError(f"{label}.skill_id is required")
                tasks.append(Task(
                    task_id, kind,
                    transit_pose=_pose(entry.get("transit_pose"),
                                       f"{label}.transit_pose"),
                    approach_pose=_pose(entry.get("approach_pose"),
                                        f"{label}.approach_pose"),
                    skill_id=skill_id, timeout=timeout, max_retries=retries,
                    end_pose=_pose(entry.get("end_pose"), f"{label}.end_pose"),
                    **limits,
                ))
        return cls(data["start_area"], "map", tasks, return_pose)

    @property
    def current(self):
        return self.tasks[self.index] if self.index < len(self.tasks) else None

    def complete_current(self):
        if self.current is None:
            raise RuntimeError("no current task")
        self.completed.append(self.current.id)
        self.index += 1
#完成任务后计数
