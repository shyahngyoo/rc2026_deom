"""Non-actuating profile diagnosis. Live checks never grant motor authority."""
import argparse
import json
from .profile import DEFAULT_CONFIG, load_profile


def static_report(config_dir,mode='localization'):
    try:
        profile,folder=load_profile(config_dir,mode)
    except Exception as error:
        return dict(status='NOT_READY',configuration_valid=False,mission_allowed=False,motion_enabled=False,
                    reasons=['PROFILE_INVALID:'+str(error)],mode=mode)
    return dict(status='NOT_READY',configuration_valid=True,config_dir=str(folder),mode=mode,
        localization_ready=False,planning_only=mode=='navigation',motion_enabled=False,mission_allowed=False,
        checks=dict(profile=True,real_low_level_state=False,real_skill_executor=False,motor_path=False),
        reasons=['REAL_LOW_LEVEL_STATE_NOT_CONNECTED','REAL_SKILL_EXECUTOR_NOT_CONNECTED','MOTION_OUTPUT_DISABLED'],
        localization_source=profile.get('localization_source','gicp'),
        absolute_drift_correction=profile.get('localization_source','gicp')=='gicp',
        frames=profile['frames'],nav_output_topic=profile['nav_output_topic'],
        note='Configuration is valid; live sensor/localization readiness is checked by doctor_node. No Mission or robot output is started.')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir',default=DEFAULT_CONFIG)
    parser.add_argument('--mode',choices=['lidar','odometry','localization','navigation'],default='localization')
    args=parser.parse_args(argv)
    report=static_report(args.config_dir,args.mode)
    print(json.dumps(report,indent=2))
    return 0 if report['configuration_valid'] else 2
