"""Load the installed BT plugin even from a terminal with stale library paths."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from launch.actions import AppendEnvironmentVariable


def test_nav2_launch_makes_installed_bt_plugin_loadable():
    path = Path(__file__).resolve().parents[1] / 'launch/nav2.launch.py'
    spec = importlib.util.spec_from_file_location('nav2_plugin_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    library_directory = os.path.join(get_package_prefix('go2_nav_bringup'), 'lib')
    context = LaunchContext()
    old_paths = [p for p in context.environment.get('LD_LIBRARY_PATH', '').split(os.pathsep)
                 if p and os.path.normpath(p) != os.path.normpath(library_directory)]
    context.environment['LD_LIBRARY_PATH'] = os.pathsep.join(old_paths)
    action = next(a for a in description.entities if isinstance(a, AppendEnvironmentVariable))
    action.execute(context)
    assert context.environment['LD_LIBRARY_PATH'].split(os.pathsep) == [library_directory, *old_paths]
    subprocess.run([
        sys.executable, '-c',
        'import ctypes; library = ctypes.CDLL("libpause_replanning_near_goal_bt_node.so"); '
        'getattr(library, "BT_RegisterNodesFromPlugin")',
    ], env=context.environment, check=True, capture_output=True, text=True, timeout=10)
