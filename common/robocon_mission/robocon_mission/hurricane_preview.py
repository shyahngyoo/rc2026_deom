"""Inspect migrated skills and UART frames without ROS or any hardware access."""
import argparse
import json
from pathlib import Path

from .hurricane_protocol import COMMAND_MAP, encode_command
from .hurricane_skills import load_skills


def main(args=None):
    parser = argparse.ArgumentParser(description="预览 Hurricane 技能；不会打开串口或发送动作")
    parser.add_argument("--skills-file", default="")
    parser.add_argument("--skill", default="")
    options = parser.parse_args(args)
    path = Path(options.skills_file) if options.skills_file else None
    if path is None:
        source = Path(__file__).resolve().parents[1] / "config/hurricane_skills.yaml"
        if source.is_file():
            path = source
        else:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("robocon_mission")) / "config/hurricane_skills.yaml"
    skills = load_skills(path)
    if options.skill and options.skill not in skills:
        parser.error("unknown skill: " + options.skill)
    selected = {options.skill: skills[options.skill]} if options.skill else skills
    output = {"dry_run": True, "protocol": "S<number>E\\r\\n", "baudrate": 115200,
              "commands": dict(COMMAND_MAP), "skills": selected,
              "completion": "requires measured pose and correlated controller feedback"}
    for skill in selected.values():
        for step in skill["steps"]:
            if "command" in step:
                step["preview_frame"] = encode_command(step["command"], step["repeat"]).decode("ascii")
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
