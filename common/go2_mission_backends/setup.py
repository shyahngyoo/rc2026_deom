from glob import glob
from setuptools import find_packages, setup
package_name = "go2_mission_backends"
setup(name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]),
 data_files=[("share/ament_index/resource_index/packages", ["resource/"+package_name]),
 ("share/"+package_name,["package.xml", "README.md"]),
 ("share/"+package_name+"/launch",glob("launch/*.launch.py")),
 ("share/"+package_name+"/config",glob("config/*.yaml")+glob("config/*.xml"))],
 install_requires=["setuptools", "numpy", "scipy", "PyYAML"], zip_safe=True,
 maintainer="ice", maintainer_email="ice@example.com", license="Apache-2.0",
 description="Mission backends with simulation-only verification fixtures", tests_require=["pytest"],
 entry_points={"console_scripts":[
 "robot_state_adapter = go2_mission_backends.robot_state_adapter:main",
 "odometry_localization = go2_mission_backends.odometry_localization_node:main",
 "localization_adapter = go2_mission_backends.localization_adapter:main",
 "motion_arbiter = go2_mission_backends.motion_arbiter:main",
 "obstacle_adapter = go2_mission_backends.obstacle_adapter:main",
 "simulation_backend = go2_mission_backends.simulation_backend:main",
 "preflight = go2_mission_backends.preflight:main",
 ]})
