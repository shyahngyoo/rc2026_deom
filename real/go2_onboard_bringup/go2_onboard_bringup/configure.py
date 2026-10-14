"""Generate a user-owned profile from explicit measured values only."""
import argparse
import json
import sys
from .profile import DEFAULT_CONFIG, create_profile


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config-dir',default=DEFAULT_CONFIG)
    p.add_argument('--host-ip',required=True)
    p.add_argument('--lidar-ip',required=True)
    p.add_argument('--imu-in-base',required=True,type=float,nargs=6,metavar=('X','Y','Z','ROLL','PITCH','YAW'),help='Measured T_base_IMU, metres/radians')
    p.add_argument('--stage',choices=['sensor_only','localization'],default='sensor_only')
    p.add_argument('--map-pcd',help='Absolute existing real XYZ PCD')
    p.add_argument('--initial-pose',type=float,nargs=6,help='Measured IMU pose in map: x y z roll pitch yaw')
    p.add_argument('--nav-map',help='Absolute existing occupancy-grid YAML')
    p.add_argument('--footprint',type=json.loads,help='Measured [[x,y],...] footprint JSON in base_footprint')
    p.add_argument('--force',action='store_true',help='Replace only named generated profile files')
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    try:
        folder=create_profile(args.config_dir,args.host_ip,args.lidar_ip,args.imu_in_base,args.stage,
            args.map_pcd,args.initial_pose,args.nav_map,args.footprint,args.force)
    except (ValueError,KeyError,TypeError,OSError) as error:
        print(json.dumps(dict(status='CONFIGURATION_REJECTED',reason=str(error),motion_enabled=False)),file=sys.stderr)
        return 2
    print(json.dumps(dict(status='CONFIGURED',config_dir=str(folder),motion_enabled=False,mission_allowed=False,
        next_command='ros2 launch go2_onboard_bringup real.launch.py config_dir:='+str(folder)+' mode:='+('localization' if args.stage=='localization' else 'lidar'))))
    return 0
