"""Measured external profiles. Configuration never authorizes robot motion."""
import ipaddress
import json
import math
from pathlib import Path
import os
import yaml

MODES = ('lidar', 'odometry', 'localization', 'navigation')
FRAMES = dict(map='map', source_odom='odom', source_imu='livox_frame',
              nav_odom='nav_odom', footprint='base_footprint', base='base_link')
FILES = ('mid360.json','fast_lio.yaml','gicp.yaml','robot_state.yaml',
         'localization.yaml','nav2.yaml','scan.yaml','preflight_route.yaml')
DEFAULT_CONFIG = str(Path.home()/'.config/rc2026')


def vector(value, size, name):
    if not isinstance(value,(list,tuple)) or len(value)!=size:
        raise ValueError(name+' must contain '+str(size)+' numbers')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value):
        raise ValueError(name+' must contain finite numbers')
    return [float(v) for v in value]


def ipv4(value, name):
    if not isinstance(value,str):raise ValueError(name+' must be an IPv4 string')
    try: ip=ipaddress.IPv4Address(value)
    except (ipaddress.AddressValueError,TypeError):raise ValueError(name+' must be an IPv4 address')
    if ip.is_multicast or ip.is_unspecified or ip.is_loopback or int(ip)==2**32-1:
        raise ValueError(name+' must be a unicast sensor-network address')
    return str(ip)


def footprint(value):
    if not isinstance(value,list) or len(value)<3:raise ValueError('footprint must have at least three measured XY vertices')
    points=[vector(p,2,'footprint vertex') for p in value]
    if len({tuple(p) for p in points})!=len(points):raise ValueError('footprint vertices must be distinct')
    area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))
    if abs(area)<1e-8:raise ValueError('footprint area must be nonzero')
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    for i in range(len(points)):
        for j in range(i+1,len(points)):
            if j==i+1 or (i==0 and j==len(points)-1):continue
            a,b=points[i],points[(i+1)%len(points)];c,d=points[j],points[(j+1)%len(points)]
            if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0:
                raise ValueError('footprint must not self-intersect')
    return points


def existing_file(value, name):
    if not isinstance(value,str) or not value or not Path(value).is_absolute():
        raise ValueError(name+' must be an absolute file path')
    p=Path(value)
    if not p.is_file() or p.stat().st_size==0:raise ValueError(name+' file missing or empty: '+value)
    return str(p.resolve())


def pcd_map(value):
    path=existing_file(value,'map_pcd')
    if Path(path).suffix.lower()!='.pcd':raise ValueError('map_pcd must have .pcd suffix')
    with Path(path).open('rb') as stream:header=stream.read(8192).decode('ascii',errors='ignore').splitlines()
    fields=next((line.split()[1:] for line in header if line.startswith('FIELDS ')),[])
    data=next((line.split()[1:] for line in header if line.startswith('DATA ')),[])
    count=next((line.split()[1:] for line in header if line.startswith('POINTS ')),[])
    if not {'x','y','z'}<=set(fields) or data not in [['ascii'],['binary'],['binary_compressed']]:
        raise ValueError('map_pcd lacks XYZ PCD header')
    try:valid=len(count)==1 and int(count[0])>0
    except ValueError:valid=False
    if not valid:raise ValueError('map_pcd must contain points')
    return path


def navigation_map(value):
    path=existing_file(value,'nav_map');data=yaml.safe_load(Path(path).read_text())
    if not isinstance(data,dict) or not isinstance(data.get('image'),str):raise ValueError('nav_map needs image')
    image=Path(data['image'])
    if not image.is_absolute():image=Path(path).parent/image
    existing_file(str(image.resolve()),'nav_map image')
    if isinstance(data.get('resolution'),bool) or not isinstance(data.get('resolution'),(int,float)) or not math.isfinite(data['resolution']) or data['resolution']<=0:
        raise ValueError('nav_map resolution must be positive')
    vector(data.get('origin'),3,'nav_map origin')
    return path


def validate_profile(profile,mode='lidar'):
    if mode not in MODES:raise ValueError('unsupported mode: '+mode)
    if not isinstance(profile,dict) or (type(profile.get('schema_version')) is not int or profile.get('schema_version')!=1):raise ValueError('profile schema_version must be 1')
    host=ipv4(profile.get('host_ip'),'host_ip');lidar=ipv4(profile.get('lidar_ip'),'lidar_ip')
    if host==lidar:raise ValueError('host_ip and lidar_ip must differ')
    ext=profile.get('imu_in_base')
    if not isinstance(ext,dict):raise ValueError('measured imu_in_base required')
    vector(ext.get('xyz'),3,'imu_in_base xyz');vector(ext.get('rpy'),3,'imu_in_base rpy')
    if profile.get('frames')!=FRAMES:raise ValueError('source/navigation frames must match FAST-LIO and adapter contracts')
    if profile.get('use_sim_time') is not False:raise ValueError('onboard use_sim_time must be false')
    if profile.get('motion_output_enabled') is not False or profile.get('mission_autostart') is not False:
        raise ValueError('this profile cannot authorize motors or Mission autostart')
    if profile.get('nav_output_topic')!='/onboard/nav_cmd_raw':raise ValueError('Nav2 output must remain isolated')
    source=profile.get('localization_source','gicp')
    if source not in ('gicp','odometry'):raise ValueError('unsupported localization_source')
    if source=='odometry':
        vector(profile.get('odom_in_map'),6,'measured odom_in_map XYZ/RPY')
        if profile.get('map_pcd') is not None or profile.get('initial_pose') is not None:
            raise ValueError('odometry mode uses odom_in_map, not GICP map_pcd/initial_pose')
    if profile.get('map_pcd') is not None:pcd_map(profile['map_pcd'])
    if profile.get('initial_pose') is not None:vector(profile['initial_pose'],6,'initial_pose IMU in map')
    if mode in ('localization','navigation') and source=='gicp':
        pcd_map(profile.get('map_pcd'));vector(profile.get('initial_pose'),6,'measured initial_pose IMU in map')
    if profile.get('footprint') is not None:footprint(profile['footprint'])
    if profile.get('nav_map') is not None:navigation_map(profile['nav_map'])
    if mode=='navigation':
        footprint(profile.get('footprint'));navigation_map(profile.get('nav_map'))
    return profile


def no_sim_time(data):
    if isinstance(data,dict):
        for key,value in data.items():
            if key=='use_sim_time' and value is not False:raise ValueError('generated real parameters must use wall time')
            no_sim_time(value)
    elif isinstance(data,list):
        for item in data:no_sim_time(item)


def load_profile(config_dir,mode='lidar'):
    folder=Path(config_dir).expanduser().resolve()
    profile_file=folder/'profile.yaml'
    if not profile_file.is_file():raise ValueError('configure_robot must create profile.yaml first: '+str(profile_file))
    profile=validate_profile(yaml.safe_load(profile_file.read_text()),mode)
    for name in FILES:existing_file(str(folder/name),'profile '+name)
    driver=json.loads((folder/'mid360.json').read_text())
    network=driver['MID360']['host_net_info']
    if any(network[k]!=profile['host_ip'] for k in ['cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip']):raise ValueError('MID360 host IP does not match profile')
    if driver['lidar_configs'][0]['ip']!=profile['lidar_ip']:raise ValueError('MID360 lidar IP does not match profile')
    params={name:yaml.safe_load((folder/name).read_text()) for name in FILES if name.endswith('.yaml')}
    for item in params.values():no_sim_time(item)
    state=params['robot_state.yaml']['robot_state_adapter']['ros__parameters']
    if (state.get('source_odom_frame')!='odom' or state.get('source_imu_frame')!='livox_frame' or
        state.get('nav_odom_frame')!='nav_odom' or state.get('footprint_frame')!='base_footprint' or state.get('base_frame')!='base_link' or
        state.get('map_frame')!='map' or state.get('odom_topic')!='/odom' or state.get('source_topic')!='/Odometry'):
        raise ValueError('robot_state source/navigation frame/topic mismatch')
    if state.get('imu_in_base_xyz')!=profile['imu_in_base']['xyz'] or state.get('imu_in_base_rpy')!=profile['imu_in_base']['rpy']:
        raise ValueError('robot_state extrinsics must match measured profile')
    lio=params['fast_lio.yaml']['/**']['ros__parameters']['common']
    if lio.get('lid_topic')!='/livox/lidar' or lio.get('imu_topic')!='/livox/imu':
        raise ValueError('FAST-LIO real sensor topics must match the configured MID360 driver')
    gicp=params['gicp.yaml']['gicp_localizer']['ros__parameters']
    if (gicp.get('map_frame')!='map' or gicp.get('odom_frame')!='odom' or gicp.get('body_frame')!='livox_frame' or
        gicp.get('odom_topic')!='/Odometry' or gicp.get('cloud_topic')!='/cloud_registered_body'):
        raise ValueError('GICP source frame/topic mismatch')
    if profile.get('localization_source','gicp')=='gicp' and mode in ('localization','navigation') and (gicp.get('initial_pose_enabled') is not True or gicp.get('initial_pose')!=profile['initial_pose']):
        raise ValueError('GICP initialization must use the measured fixed-start pose')
    local=params['localization.yaml']['localization_evidence_adapter']['ros__parameters']
    evidence_topic='/odometry/evidence' if profile.get('localization_source')=='odometry' else '/gicp/evidence'
    if local.get('map_frame')!='map' or local.get('body_frame')!='livox_frame' or local.get('input_topic')!=evidence_topic or local.get('output_topic')!='/localization/evidence' or local.get('health_service')!='/mission/localization_ok':
        raise ValueError('localization atomic evidence/frame mismatch')
    if mode=='navigation':
        nav=params['nav2.yaml'];expected=json.dumps(profile['footprint'])
        for key,frame in [('local_costmap','nav_odom'),('global_costmap','map')]:
            p=nav[key][key]['ros__parameters']
            if p.get('global_frame')!=frame or p.get('robot_base_frame')!='base_footprint' or p.get('footprint')!=expected or 'robot_radius' in p:
                raise ValueError('Nav2 requires measured footprint and navigation frames')
        for node in ['controller_server','bt_navigator','velocity_smoother']:
            p=nav[node]['ros__parameters']
            if node!='controller_server' and p.get('odom_topic')!='/odom':raise ValueError('Nav2 odom_topic mismatch')
    return profile,folder


def template_directory():
    local=Path(__file__).resolve().parents[1]/'config'
    if local.is_dir():return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('go2_onboard_bringup'))/'config'


def create_profile(config_dir,host_ip,lidar_ip,imu_in_base,stage='sensor_only',map_pcd=None,initial_pose=None,nav_map=None,measured_footprint=None,force=False,localization_source='gicp',odom_in_map=None):
    if stage not in ('sensor_only','localization'):raise ValueError('stage must be sensor_only or localization')
    ext=vector(imu_in_base,6,'imu_in_base XYZ/RPY')
    profile=dict(schema_version=1,host_ip=host_ip,lidar_ip=lidar_ip,
        imu_in_base=dict(xyz=ext[:3],rpy=ext[3:]),frames=FRAMES.copy(),map_pcd=map_pcd,
        initial_pose=initial_pose,nav_map=nav_map,footprint=measured_footprint,
        localization_source=localization_source,odom_in_map=odom_in_map,
        use_sim_time=False,motion_output_enabled=False,mission_autostart=False,nav_output_topic='/onboard/nav_cmd_raw')
    validate_profile(profile,'localization' if stage=='localization' else 'lidar')
    folder=Path(config_dir).expanduser().resolve()
    if not force and any((folder/name).exists() for name in ('profile.yaml',*FILES)):
        raise ValueError('profile files already exist; use --force to replace these named files')
    source=template_directory()
    documents={name:yaml.safe_load((source/(name.replace('.yaml','.template.yaml'))).read_text()) for name in ['fast_lio.yaml','gicp.yaml','robot_state.yaml','localization.yaml','nav2.yaml','scan.yaml']}
    gicp=documents['gicp.yaml']['gicp_localizer']['ros__parameters']
    gicp['initial_pose_enabled']=initial_pose is not None
    gicp['initial_pose']=initial_pose if initial_pose is not None else [0.]*6
    if localization_source=='odometry':
        local=documents['localization.yaml']['localization_evidence_adapter']['ros__parameters']
        local.update(input_topic='/odometry/evidence',evidence_max_age=.5)
    state=documents['robot_state.yaml']['robot_state_adapter']['ros__parameters']
    state.update(imu_in_base_xyz=ext[:3],imu_in_base_rpy=ext[3:])
    for doc in documents.values():
        # Generated data is always explicitly real-clock only.
        def mark(x):
            if isinstance(x,dict):
                if 'ros__parameters' in x:x['ros__parameters']['use_sim_time']=False
                for v in list(x.values()):mark(v)
        mark(doc)
    if measured_footprint is not None:
        for key in ['local_costmap','global_costmap']:
            documents['nav2.yaml'][key][key]['ros__parameters']['footprint']=json.dumps(measured_footprint)
    pose=odom_in_map if localization_source=='odometry' else initial_pose or [0.]*6
    documents['preflight_route.yaml']=dict(start_area='readiness_only_not_executed',frame_id='map',
        route=[dict(id='diagnostic_only',type='navigation',pose=[pose[0],pose[1],pose[5]])],
        return_=None)
    documents['preflight_route.yaml'].pop('return_')
    documents['preflight_route.yaml']['return']=dict(pose=[pose[0],pose[1],pose[5]])
    driver=json.loads((source/'mid360.template.json').read_text())
    for key in ['cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip']:driver['MID360']['host_net_info'][key]=host_ip
    driver['lidar_configs'][0]['ip']=lidar_ip
    files={name:yaml.safe_dump(doc,sort_keys=False) for name,doc in documents.items()}
    files['mid360.json']=json.dumps(driver,indent=2)+'\n';files['profile.yaml']=yaml.safe_dump(profile,sort_keys=False)
    folder.mkdir(parents=True,exist_ok=True)
    for name,text in files.items():
        temporary=folder/(name+'.tmp');temporary.write_text(text);os.replace(temporary,folder/name)
    load_profile(folder,'localization' if stage=='localization' else 'lidar')
    return folder
