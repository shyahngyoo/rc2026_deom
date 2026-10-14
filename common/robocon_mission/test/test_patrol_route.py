"""Check the shipped patrol's corridors against the shipped occupancy map."""

from functools import lru_cache
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from robocon_mission.mission_fsm import MissionFSM, State
from robocon_mission.route_manager import RouteManager


def patrol():
    return RouteManager.load(Path(__file__).resolve().parents[1] /
                             "config" / "competition_route.yaml")


def test_patrol_visits_all_main_regions_and_returns():
    route = patrol()
    assert all(task.type == "navigation" for task in route.tasks)
    assert min(task.pose[0] for task in route.tasks) <= -5.0
    assert max(task.pose[0] for task in route.tasks) >= 5.0
    assert min(task.pose[1] for task in route.tasks) <= -4.0
    assert max(task.pose[1] for task in route.tasks) >= 4.0
    assert route.return_pose == (0.0, 0.0, 0.0)
    fsm = MissionFSM(route, 120.0, 90.0, 20.0, 1, True)
    command = fsm.start()
    command = fsm.result(command.token, "succeeded")
    assert command.kind == "handoff" and command.mode == "NAVIGATION"
    command = fsm.result(command.token, "succeeded", confirmed_owner="NAVIGATION")
    for task in route.tasks:
        assert command.kind == "navigation" and command.task_id == task.id
        assert command.pose == task.pose
        command = fsm.result(command.token, "succeeded")
    assert fsm.state == State.RETURNING and command.pose == route.return_pose
    assert fsm.result(command.token, "succeeded") is None
    assert fsm.state == State.FINISHED
    assert route.completed == [task.id for task in route.tasks]


def test_patrol_segments_have_clearance_from_static_obstacles():
    map_dir = Path(__file__).resolve().parent / "fixtures" / "patrol_map"
    meta = yaml.safe_load((map_dir / "mid360_3d_nav.yaml").read_text())
    with (map_dir / meta["image"]).open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        dimensions = []
        while len(dimensions) < 3:
            line = stream.readline().split(b"#", 1)[0]
            dimensions.extend(int(value) for value in line.split())
        width, height, maximum = dimensions
        assert maximum == 255
        pixels = stream.read()
    assert len(pixels) == width * height
    assert meta["negate"] == 0 and meta["origin"][2] == 0.0
    resolution = meta["resolution"]
    origin_x, origin_y, _ = meta["origin"]
    # Above the modeled 0.22 m radius; include room for transit goal error.
    clearance = 0.55
    radius = math.ceil(clearance / resolution)
    offsets = [(dx, dy) for dx in range(-radius, radius + 1)
               for dy in range(-radius, radius + 1)
               if math.hypot(dx, dy) * resolution <= clearance]

    @lru_cache(None)
    def clear(col, row):
        for dx, dy in offsets:
            x, y = col + dx, row + dy
            if not (0 <= x < width and 0 <= y < height):
                return False
            occupied = 1.0 - pixels[(height - 1 - y) * width + x] / 255.0
            if occupied >= meta["free_thresh"]:
                return False
        return True

    route = patrol()
    poses = [route.return_pose] + [task.pose for task in route.tasks] + [route.return_pose]
    for start, end in zip(poses, poses[1:]):
        samples = max(1, math.ceil(math.hypot(end[0] - start[0], end[1] - start[1]) /
                                   (resolution / 2.0)))
        for index in range(samples + 1):
            fraction = index / samples
            x = start[0] + fraction * (end[0] - start[0])
            y = start[1] + fraction * (end[1] - start[1])
            col = math.floor((x - origin_x) / resolution)
            row = math.floor((y - origin_y) / resolution)
            assert clear(col, row), f"patrol corridor too close to an obstacle at {(x, y)}"
