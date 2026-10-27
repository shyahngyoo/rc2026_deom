"""Desktop entry for mapping, localization/capture and CyberGear operation."""
import argparse
from datetime import datetime
from pathlib import Path
import queue
import re
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import yaml
import json
import math
import uuid
import shutil
import time
import signal
from .panel_view import build_view, status_summary
import rclpy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from .map_pose import pose_vector, write_start, read_start, prepare_cloud, PosePicker
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from .panel_support import Processes, run_command, saved_path, apply_localization
from .profile import load_profile


class Panel:
    def __init__(self, root, config):
        self.root = root
        self.config = Path(config).expanduser().resolve()
        self.node=rclpy.create_node('rc2026_operator_panel')
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self.node)
        self.key_pub=self.node.create_publisher(String,'/cybergear/key',10)
        self.capture_frame='map';self.odom_session=None;self.sensor_mode=None
        self.last_status={}
        self.mapping_start=None
        self.active_map=None
        self.match_pose=None;self.match_valid=False;self.match_valid_time=0
        self.match_text='尚未尝试定位';self.match_requested=0
        self.initial_pub=self.node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.node.create_subscription(PoseStamped,'/gicp/pose',self.on_match_pose,10)
        self.node.create_subscription(Bool,'/gicp/valid',self.on_match_valid,10)
        self.node.create_subscription(String,'/gicp/status',self.on_match_status,10)
        self.node.create_subscription(String,'/cybergear/status',self.on_status,10)
        self.events = queue.Queue()
        self.busy = False
        self.mode = None
        self.session_dir = None
        self.procs = Processes(Path.home()/'rc2026_logs'/datetime.now().strftime('panel_%Y%m%d_%H%M%S_%f'))
        profile_path = self.config/'profile.yaml'
        profile = yaml.safe_load(profile_path.read_text()) if profile_path.exists() else {}
        self.status_received = None
        self.close_requested = False
        self.worker = None
        build_view(self, profile)
        self.ensure_config()
        self.log('日志目录：'+str(self.procs.log_dir))
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(150,self.poll)

    def choose(self,key):
        path = filedialog.askopenfilename(filetypes=[('PCD','*.pcd')] if key=='map' else [('YAML','*.yaml')])
        if path:self.values[key].set(path)

    def log(self,text):
        self.text.insert('end',str(text)+'\n'); self.text.see('end')

    def dispatch(self,fn):
        if self.busy:
            self.log('正在完成上一操作，请稍候。'); return
        data={k:v.get().strip() for k,v in self.values.items()}
        data['only_keys']=self.only_keys.get()
        self.busy=True; self.state.set('正在处理，请稍候…')
        def worker():
            try:
                result=fn(data)
                self.events.put(('done',str(result or '操作完成')))
            except Exception as exc:self.events.put(('error',str(exc)))
        self.worker=threading.Thread(target=worker,daemon=True)
        self.worker.start()

    def poll(self):
        while not self.events.empty():
            kind,text=self.events.get()
            if kind=='map':self.values['map'].set(text)
            elif kind=='pose':
                self.values['pose'].set(' '.join(f'{v:.6f}' for v in text))
            elif kind=='cloud':
                path,cloud,initial=text
                if self.values['map'].get().strip()!=path:
                    self.log('地图选择已改变，请重新打开预览。')
                else:PosePicker(self.root,cloud,initial,lambda pose:self.apply_picked_pose(path,pose))
            elif kind=='close':self.node.destroy_node();self.root.destroy();return
            else:
                self.busy=False; self.state.set(text); self.log(text)
                if kind=='error' and not self.close_requested:messagebox.showerror('操作失败',text)
        if self.close_requested and not self.busy:
            self.close()
        rclpy.spin_once(self.node,timeout_sec=0)
        age=None if self.status_received is None else time.monotonic()-self.status_received
        title,hint,mode=status_summary(self.last_status,self.procs.alive('bridge'),age)
        if self.sensor_mode!='localization' or not self.procs.alive('sensors'):
            self.localization_state.set('定位：尚未运行地图定位')
        elif self.match_pose is not None and time.monotonic()-self.match_valid_time>2:
            self.localization_state.set('定位：反馈已过期，请检查雷达和定位进程')
        else:self.localization_state.set('定位：'+self.match_text)
        self.connection.set(title); self.status_hint.set(hint); self.robot_mode.set(mode)
        task=self.last_status.get('task','no_plan') if self.procs.alive('bridge') and age is not None and age<=2 else 'unknown'
        self.task_state.set({'no_plan':'未载入任务','idle':'待开始','running':'执行中','completed':'步骤已结束','stopped':'已停止','unknown':'等待状态'}.get(task,task))
        try:
            x,y,yaw=self.read_capture_pose()
            self.position.set(f'位置 [{self.capture_frame}]：x={x:.3f} m，y={y:.3f} m，yaw={math.degrees(yaw):.1f}°')
        except Exception:
            self.position.set('位置：等待新鲜定位/里程计数据')
        # Process startup is asynchronous: show unexpected exits explicitly.
        if not self.busy and self.mode:
            dead=[k for k in ('sensors','bridge') if k in self.procs.items and not self.procs.alive(k)]
            if dead:self.state.set('进程已退出：'+','.join(dead)+'；请查看日志目录')
        self.root.after(150,self.poll)

    def real(self,mode):
        self.sensor_mode=mode
        self.procs.start('sensors',['ros2','launch','go2_onboard_bringup','real.launch.py',
                                  'config_dir:='+str(self.config),'mode:='+mode])

    def rviz(self,mapping):
        pkg='go2_real_bringup' if mapping else 'go2_real_localization'
        rel='config/real_mapping.rviz' if mapping else 'rviz/localization.rviz'
        self.procs.start('rviz',['rviz2','-d',str(Path(get_package_share_directory(pkg))/rel)])

    def on_status(self,msg):
        if self.mode!='auto':
            return
        try:
            status=json.loads(msg.data)
            if not isinstance(status,dict):return
            self.last_status=status
            self.status_received=time.monotonic()
            self.live.set('就绪：'+str(status.get('ready'))+' 任务：'+str(status.get('task'))+
                          ' 障碍：'+str(status.get('obstacle',''))+' 步骤：'+str(status.get('step'))+' 动作：'+str(status.get('action',''))+' 回包：'+str(status.get('status')))
        except (ValueError,TypeError):
            pass

    def mapping(self,d):
        name=d['map_name']
        if not re.fullmatch(r'[\w-]{1,64}',name):raise ValueError('地图名称用文字、数字、下划线或短横线，最多64字')
        load_profile(self.config,'odometry')
        self.procs.stop_all(); self.mode=None
        self.session_dir=Path.home()/'rc2026_maps'/(name+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.session_dir.mkdir(parents=True)
        self.mapping_start=None
        self.real('odometry'); self.mode='mapping'; self.odom_session=None; self.rviz(True)
        # Record the first real IMU pose; ask operator to remain still until completed.
        output=run_command(['ros2','topic','echo','/Odometry','nav_msgs/msg/Odometry','--once'],45)
        messages=[item for item in yaml.safe_load_all(output) if item is not None]
        if len(messages)!=1:raise ValueError('未收到唯一的建图起点位姿，请检查 /Odometry')
        odom=messages[0]
        self.mapping_start={'pose':pose_vector(odom['pose']['pose']),'frame':odom['header']['frame_id']}
        (self.session_dir/'mapping_start_odom.yaml').write_text(output)
        self.events.put(('pose',self.mapping_start['pose']))
        return '建图起点已记录并填入位姿栏。现在可遥控走场；保存后若使用起点位姿定位，须回到同一位置和朝向。'

    def save_map(self,d):
        if self.mode!='mapping' or not self.procs.alive('sensors'):raise ValueError('请先开始建图并保持雷达运行')
        output=run_command(['ros2','run','go2_fastlio_localization','save_live_map','--ros-args',
                            '-p','output_dir:='+str(self.session_dir),'-p','timeout_sec:=45.0'],55)
        path=saved_path(output)
        self.events.put(('map',path))
        start=getattr(self,'mapping_start',None)
        if start:
            frames=[line[6:].strip() for line in output.splitlines() if line.startswith('FRAME ')]
            if frames!=[start['frame']]:
                return 'PCD 已保存，但地图坐标系与起点记录不一致，未关联起点。请图上指定初始位姿。'
            write_start(path,start['pose'],start['frame'])
        return '保存成功：'+path+'\n已自动填入定位地图栏。确认当前 IMU 地图位姿后再切到定位。'

    def prepare_localization(self,d):
        load_profile(self.config,'odometry')
        pose=[float(v) for v in d['pose'].replace(',',' ').split()]
        backup=apply_localization(self.config,d['map'],pose)
        return backup

    def localize(self,d):
        if self.procs.alive('sensors') and self.mode=='localization':
            return '定位已运行；修改地图或初始位姿后请先停止全部程序再启动。'
        backup=self.prepare_localization(d)
        self.procs.stop_all();self.mode=None
        self.active_map=str(Path(d['map']).expanduser().resolve())
        self.reset_match()
        self.real('localization');self.mode='localization';self.capture_frame='map';self.rviz(False)
        return '定位已启动，请在 RViz 核对位置后打开采点面板。配置备份：'+str(backup)

    def reset_match(self):
        self.match_pose=None;self.match_valid=False;self.match_valid_time=0
        self.match_requested=self.node.get_clock().now().nanoseconds*1e-9
        self.match_text='等待新雷达数据与匹配结果'

    def on_match_pose(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.sensor_mode=='localization' and msg.header.frame_id=='map' and stamp>self.match_requested:
            self.match_pose=msg

    def on_match_valid(self,msg):
        self.match_valid=bool(msg.data);self.match_valid_time=time.monotonic()

    def on_match_status(self,msg):
        if self.sensor_mode=='localization':
            self.match_text=msg.data.replace('ACCEPTED','匹配通过').replace('REJECTED','匹配未通过').replace('WAITING_FOR_SYNCHRONIZED_INPUT','等待雷达与里程计同步数据').replace('NOT_CONVERGED','尚未收敛').replace('STALE_INPUT','输入数据已过期')

    def apply_picked_pose(self,path,pose):
        if self.values['map'].get().strip()!=path:
            raise ValueError('主面板已更换地图，请重新打开地图预览')
        self.values['pose'].set(' '.join(f'{v:.6f}' for v in pose))
        self.log('已填入地图点选位姿；点击“尝试定位”，在 RViz 核对点云重合后再使用结果。')

    def preview_map(self,d):
        if not d['map']:raise ValueError('请先选择旧地图 PCD')
        cloud=prepare_cloud(d['map'])
        try:initial=read_start(d['map'])
        except ValueError:initial=None
        if d['pose']:
            from .profile import vector
            initial=vector([float(v) for v in d['pose'].replace(',',' ').split()],6,'初始位姿')
        self.events.put(('cloud',(d['map'],cloud,initial)))
        return '地图已打开：点击位置并拖动箭头确定朝向。'

    def use_map_start(self,d):
        if not d['map']:raise ValueError('请先选择对应的 PCD 地图')
        pose=read_start(d['map']);self.events.put(('pose',pose))
        return '已填入这张地图的建图起点。请把机器人放回当时同一位置与朝向，再尝试定位。'

    def try_localization(self,d):
        if self.procs.alive('bridge'):
            raise ValueError('请先停止任务并停止本面板全部程序，再重新定位。')
        if self.sensor_mode=='localization' and self.procs.alive('sensors'):
            if str(Path(d['map']).expanduser().resolve())!=self.active_map:
                raise ValueError('定位正在使用另一张地图，请先停止建图 / 定位，再重试。')
            from .profile import vector
            xyzrpy=vector([float(v) for v in d['pose'].replace(',',' ').split()],6,'初始位姿')
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.node.get_clock().now().to_msg()
            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=xyzrpy[:3]
            r,p,y=(v/2 for v in xyzrpy[3:]);cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
            q=msg.pose.pose.orientation
            q.x=sr*cp*cy-cr*sp*sy;q.y=cr*sp*cy+sr*cp*sy;q.z=cr*cp*sy-sr*sp*cy;q.w=cr*cp*cy+sr*sp*sy
            self.reset_match();self.initial_pub.publish(msg)
            return '已发送新初始位姿，等待匹配；请在 RViz 检查点云是否重合。'
        return self.localize(d)

    def use_match(self,d):
        msg=self.match_pose
        if self.sensor_mode!='localization' or not self.procs.alive('sensors') or msg is None:
            raise ValueError('尚未获得本次定位结果，请先尝试定位')
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.node.get_clock().now().nanoseconds*1e-9-stamp
        if not self.match_valid or time.monotonic()-self.match_valid_time>2 or not 0<=age<=2:
            raise ValueError('定位结果未通过或已过期，请重新匹配')
        if str(Path(d['map']).expanduser().resolve())!=self.active_map:
            raise ValueError('所选地图与正在定位的地图不一致')
        p=msg.pose.position;q=msg.pose.orientation
        pose=pose_vector(dict(position=dict(x=p.x,y=p.y,z=p.z),orientation=dict(x=q.x,y=q.y,z=q.z,w=q.w)))
        backup=apply_localization(self.config,d['map'],pose)
        self.events.put(('pose',pose))
        return '已保存当前匹配位姿，定位保持运行，可进入采点或任务页面。配置备份：'+str(backup)

    def capture(self,d):
        if self.mode!='localization' or not self.procs.alive('sensors'):raise ValueError('先启动“定位与采点”模式')
        path=Path(d['field']).expanduser();path.parent.mkdir(parents=True,exist_ok=True)
        self.procs.start('capture',['ros2','run','robocon_mission','capture_panel','--file',str(path)])
        return '采点面板已打开；这里只记录坐标，不自动生成障碍模板。'

    def ensure_config(self):
        self.config.mkdir(parents=True,exist_ok=True)
        source=Path(get_package_share_directory('quadruped_stm32_bridge'))/'config'
        for src,dst in [('cybergear.yaml','cybergear.yaml'),('cybergear_plan_example.yaml','cybergear_plan.yaml')]:
            target=self.config/dst
            if not target.exists():shutil.copy2(source/src,target)

    def open_course(self):
        try:
            from .course_panel import CoursePanel
            CoursePanel(self)
        except Exception as exc:messagebox.showerror('场地配置',str(exc))

    def read_capture_pose(self):
        if not self.procs.alive('sensors'):raise ValueError('请先开启快速里程计或定位与采点')
        t=self.tf.lookup_transform(self.capture_frame,'base_footprint',Time())
        stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
        age=self.node.get_clock().now().nanoseconds*1e-9-stamp
        if not 0<=age<=0.5:raise ValueError('位置数据已过期，请检查雷达/定位')
        q=t.transform.rotation
        pose=[t.transform.translation.x,t.transform.translation.y,
              math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        if not all(math.isfinite(v) for v in pose):raise ValueError('位置数据无效')
        return pose

    def quick_odometry(self,d):
        load_profile(self.config,'odometry')
        self.procs.stop_all();self.mode=None
        self.real('odometry');self.mode='odometry';self.capture_frame='nav_odom'
        self.odom_session=uuid.uuid4().hex
        self.rviz(True)
        return '快速里程计已启动。保持静止待初始化后打开场地窗口采点；本次坐标只在本次里程计运行期间有效。'

    def manual_key(self,key):
        if not self.procs.alive('bridge'):
            self.log('先点“连接串口单动作调试”或载入自动任务');return
        if key!=5 and (not self.last_status.get('ready') or self.last_status.get('task')=='running'):
            self.log('等待串口就绪，且先停止自动任务再调单键');return
        if key!=5 and self.last_status.get('pending_mode'):
            self.log('等待模式反馈确认后再发动作');return
        self.key_pub.publish(String(data=str(key)))
        self.log('发送按键 '+str(key)+'（非动作完成反馈）')

    def launch_bridge(self,d,plan=''):
        self.ensure_config()
        if not d['port']:raise ValueError('请填写串口设备')
        self.procs.stop('bridge');self.last_status={};self.status_received=None
        self.procs.start('bridge',['ros2','launch','go2_onboard_bringup','cybergear_auto.launch.py',
            'port:='+d['port'],'enable_motion:=true','params_file:='+str(self.config/'cybergear.yaml'),'plan_file:='+str(plan)])
        self.mode='auto'

    def debug_bridge(self,d):
        self.launch_bridge(d)
        return '单动作调试已连接；遥控器打开自动，先发模式键并等待回包，再点动作按钮。'

    def automatic(self,d):
        from quadruped_stm32_bridge.cybergear_core import validate_plan
        plan=Path(d['plan']).expanduser().resolve()
        data=yaml.safe_load(plan.read_text());steps=validate_plan(data)
        has_pose=any(s['type'] in ('waypoint','local_waypoint') for s in steps)
        if d['only_keys'] and has_pose:raise ValueError('任务包含航点，不能勾选只调按键')
        self.procs.stop('bridge');self.procs.stop('capture')
        frame=data.get('frame','map')
        if has_pose:
            if frame=='nav_odom':
                if (self.sensor_mode!='odometry' or not self.procs.alive('sensors')
                        or not self.odom_session or data.get('course_session')!=self.odom_session):
                    raise ValueError('快速里程计已重启或不是本次采点，请重新采点生成任务')
                self.capture_frame='nav_odom'
            elif frame=='map':
                if self.sensor_mode!='localization' or not self.procs.alive('sensors'):
                    self.procs.stop('rviz');self.procs.stop('sensors')
                    self.prepare_localization(d);self.real('localization');self.rviz(False)
                self.capture_frame='map'
            else:raise ValueError('未知任务坐标系')
        self.launch_bridge(d,plan)
        return '任务已载入，等待串口和定位就绪后点“开始任务”。采点使用的定位进程保持运行。'

    def start_task(self,d):
        if self.mode!='auto' or not self.procs.alive('bridge'):raise ValueError('请先启动自动运行模式')

        return run_command(['ros2','service','call','/cybergear/start','std_srvs/srv/Trigger','{}'],8)

    def stop_task(self,d):
        return run_command(['ros2','service','call','/cybergear/stop','std_srvs/srv/Trigger','{}'],8)

    def stop_all(self,d):
        self.procs.stop_all();self.mode=None;self.sensor_mode=None;self.odom_session=None
        self.last_status={};self.status_received=None
        return '本面板启动的程序已停止；未终止其他终端的程序。'

    def stop_sensors(self,d):
        if self.procs.alive('bridge'):
            raise ValueError('串口 / 自动任务仍在运行。请先停止任务，再点“停止本面板全部程序”；不能在任务中切断定位。')
        for name in ('capture','rviz','sensors'):
            self.procs.stop(name)
        self.mode=None; self.sensor_mode=None; self.odom_session=None
        return '建图 / 定位已停止，面板保留。可重新开始建图；旧地图文件未删除。'

    def request_close(self):
        self.close_requested=True
        if self.busy:
            self.state.set('退出已请求，等待当前操作结束后清理后台程序…')
        else:
            self.close()

    def close(self):
        if self.busy:
            messagebox.showinfo('请稍候','当前操作结束后再关闭；地图保存最多约55秒。');return
        self.busy=True
        def worker():
            self.procs.stop_all();self.events.put(('close',''))
        self.worker=threading.Thread(target=worker,daemon=True)
        self.worker.start()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir',default=str(Path.home()/'.config/rc2026'))
    args=parser.parse_args()
    rclpy.init(args=[])
    panel=None
    previous={}
    try:
        root=tk.Tk();panel=Panel(root,args.config_dir)
        for sig in (signal.SIGINT,signal.SIGTERM):
            previous[sig]=signal.signal(sig,lambda *_:root.after(0,panel.request_close))
        root.mainloop()
    finally:
        if panel is not None:
            if panel.worker is not None:panel.worker.join()
            panel.procs.stop_all()
        for sig,handler in previous.items():signal.signal(sig,handler)
        if rclpy.ok():rclpy.shutdown()
