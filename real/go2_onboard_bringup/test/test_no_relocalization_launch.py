import sys
from pathlib import Path
import pytest
pytest.importorskip('launch_ros')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_onboard_bringup import launching


@pytest.mark.parametrize('source', ['gicp', 'odometry'])
def test_only_one_localization_source_is_launched(monkeypatch, tmp_path, source):
    profile = dict(localization_source=source, map_pcd='/measured/map.pcd', odom_in_map=[0.]*6)
    monkeypatch.setattr(launching, 'load_profile', lambda *_: (profile, tmp_path))
    monkeypatch.setattr(launching, 'include', lambda package, filename, arguments: dict(package=package, file=filename))
    monkeypatch.setattr(launching, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(launching, 'planning_actions', lambda _: [])
    actions = [a for a in launching.onboard_actions(tmp_path, 'navigation') if isinstance(a, dict)]
    files = [a['file'] for a in actions if 'file' in a]
    executables = [a['executable'] for a in actions if 'executable' in a]
    assert ('localization.launch.py' in files) == (source == 'gicp')
    assert ('odometry_localization' in executables) == (source == 'odometry')
    if source == 'odometry':
        assert files == ['lidar.launch.py', 'odometry.launch.py']
    assert 'robot_state_adapter' in executables and 'localization_adapter' in executables
    assert 'mission_manager' not in executables
