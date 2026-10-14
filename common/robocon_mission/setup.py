from glob import glob
from setuptools import find_packages, setup


package_name = "robocon_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.xml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
        ("share/" + package_name + "/docs", glob("docs/*.png")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/docs/verification", glob("docs/verification/*.log")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="icecream127",
    maintainer_email="icecream127@users.noreply.github.com",
    description="Finite-state ROBOCON mission scheduler for Nav2 and obstacle skills",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mission_manager = robocon_mission.mission_manager:main",
        "mock_localization_ok = robocon_mission.mock_localization_ok:main",
        "localization_health = robocon_mission.localization_health:main",
        "mock_motion_services = robocon_mission.mock_motion_services:main",
        "mock_mission_demo = robocon_mission.mock_mission_demo:main",
    ]},
)
