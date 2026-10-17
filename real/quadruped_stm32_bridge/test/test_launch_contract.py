"""Launch/config contracts for the default non-actuating integration."""
import importlib.util
from pathlib import Path
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters


def load():
    path=Path(get_package_share_directory('quadruped_stm32_bridge'))/'launch/stm32_bringup.launch.py'
    spec=importlib.util.spec_from_file_location('stm32_launch_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    for a in module.generate_launch_description().entities:
        if isinstance(a,DeclareLaunchArgument):a.execute(context)
    return module,context


def test_default_launch_cannot_enable_uart_motion_or_start_skill_action():
    module,context=load()
    assert context.launch_configurations['enable_motion']=='false'
    assert context.launch_configurations['test_mode']=='true'
    assert context.launch_configurations['calibration_confirmed']=='false'
    assert context.launch_configurations['port']==''
    nodes=module._nodes(context)[1:]
    assert len(nodes)==4
    parameters=[evaluate_parameters(context,n._Node__parameters) for n in nodes]
    gateway=parameters[0][-1]
    assert not gateway['enable_motion'] and gateway['test_mode']
    arbiter=parameters[-1][-1]
    assert not arbiter['enable_output'] and not arbiter['simulation_mode']
    assert all(not p[-1]['use_sim_time'] for p in parameters)


@pytest.mark.parametrize('values', [dict(enable_motion='true'),
    dict(enable_motion='true',test_mode='false'),dict(start_hurricane='true'),
    dict(enable_motion='true',test_mode='false',calibration_confirmed='true')])
def test_incomplete_hardware_enable_is_rejected(values):
    module,context=load();context.launch_configurations.update(values)
    with pytest.raises(ValueError):module._nodes(context)


def test_real_arbiter_has_private_output_and_no_simulator():
    path=Path(get_package_share_directory('go2_mission_backends'))/'config/arbiter_real.yaml'
    p=yaml.safe_load(path.read_text())['motion_arbiter']['ros__parameters']
    assert p['output_topic']=='/stm32/arbiter_cmd_vel'
    assert p['enable_output'] is False and p['simulation_mode'] is False
    assert p['telemetry_source_id']=='stm32'
    assert p['command_timeout']>0 and p['state_timeout']>0


def test_all_nav_controllers_and_smoother_disable_lateral_motion():
    repo=Path(__file__).resolve().parents[3]
    for relative in ('real/go2_onboard_bringup/config/nav2.template.yaml',
                     'common/go2_nav_bringup/config/autonomy/fast_lio_nav2.yaml'):
        cfg=yaml.safe_load((repo/relative).read_text())
        controllers=cfg['controller_server']['ros__parameters']
        for name in controllers['controller_plugins']:
            c=controllers[name]
            for key in ('min_vel_y','max_vel_y','acc_lim_y','decel_lim_y'):
                assert c[key]==0.0
            assert c['vy_samples']==1
        assert controllers['FollowPathApproach']['PrecisionPose.holonomic'] is False
        for key in ('max_velocity','min_velocity','max_accel','max_decel','deadband_velocity'):
            assert cfg['velocity_smoother']['ros__parameters'][key][1]==0.0


def test_hurricane_gateway_timeout_uses_the_declared_parameter_name():
    path=Path(get_package_share_directory('quadruped_stm32_bridge'))/'config/stm32.yaml'
    p=yaml.safe_load(path.read_text())['hurricane_skill_server']['ros__parameters']
    assert p['gateway_status_timeout']==0.3
    assert 'status_timeout' not in p
