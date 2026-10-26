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
import rclpy
from std_msgs.msg import String
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
        self.node.create_subscription(String,'/cybergear/status',self.on_status,10)
        self.events = queue.Queue()
        self.busy = False
        self.mode = None
        self.session_dir = None
        self.procs = Processes(Path.home()/'rc2026_logs'/datetime.now().strftime('panel_%Y%m%d_%H%M%S_%f'))
        root.title('RC2026 建图 / 采点 / CyberGear 自动运行')
        root.geometry('1050x850')
        profile_path = self.config/'profile.yaml'
        profile = yaml.safe_load(profile_path.read_text()) if profile_path.exists() else {}
        self.values = {}
        form = ttk.Frame(root, padding=10); form.pack(fill='x')
        fields = [('map_name','本次建图名称','field'),
                  ('map','定位 PCD',profile.get('map_pcd') or ''),
                  ('pose','当前 IMU 地图位姿 x y z roll pitch yaw',
                   ' '.join(map(str,profile.get('initial_pose') or []))),
                  ('port','串口设备','/dev/ttyUSB0'),
                  ('plan','按键任务 YAML',str(self.config/'cybergear_plan.yaml')),
                  ('field','采点文件',str(Path.home()/'rc2026_field/field.yaml'))]
        for i, (key, label, value) in enumerate(fields):
            ttk.Label(form,text=label).grid(row=i,column=0,sticky='w')
            self.values[key] = tk.StringVar(value=value)
            ttk.Entry(form,textvariable=self.values[key],width=78).grid(row=i,column=1,sticky='ew')
            if key in ('map','plan'):
                ttk.Button(form,text='选择文件',command=lambda k=key:self.choose(k)).grid(row=i,column=2)
        form.columnconfigure(1,weight=1)
        ttk.Label(root,text='已有 IP/外参/轮廓保留。先关闭在其他终端运行的雷达、定位和运动程序。').pack(anchor='w',padx=10)
        bar = ttk.Frame(root,padding=10); bar.pack(fill='x')
        actions = [('① 开始建图',self.mapping),('② 保存地图',self.save_map),
                   ('③ 定位与采点',self.localize),('旧采点面板',self.capture),
                   ('④ 自动运行模式',self.automatic),('开始任务',self.start_task),
                   ('停止任务',self.stop_task),('停止本面板全部程序',self.stop_all)]
        for i,(label,fn) in enumerate(actions):
            ttk.Button(bar,text=label,command=lambda f=fn:self.dispatch(f)).grid(row=i//4,column=i%4,padx=4,pady=4,sticky='ew')
        extra=ttk.Frame(root,padding=8);extra.pack(fill='x')
        ttk.Button(extra,text='快速里程计（无需地图）',command=lambda:self.dispatch(self.quick_odometry)).pack(side='left')
        ttk.Button(extra,text='场地 / 障碍编排与采点',command=self.open_course).pack(side='left',padx=4)
        ttk.Button(extra,text='连接串口单动作调试',command=lambda:self.dispatch(self.debug_bridge)).pack(side='left',padx=4)
        controls=ttk.Frame(root,padding=8);controls.pack(fill='x')
        self.debug_key=tk.StringVar(value='9')
        ttk.Combobox(controls,textvariable=self.debug_key,values=[str(k) for k in range(9,19)],width=5,state='readonly').pack(side='left')
        ttk.Button(controls,text='发送模式键',command=lambda:self.manual_key(int(self.debug_key.get()))).pack(side='left')
        for label,key in [('左转',1),('前进',2),('右转',3),('后退',4),('停止',5),('动作6',6),('动作7',7),('动作8',8)]:
            ttk.Button(controls,text=label,command=lambda k=key:self.manual_key(k)).pack(side='left')
        self.only_keys = tk.BooleanVar(value=False)
        ttk.Checkbutton(root,text='只调按键 / 时序（自动运行模式不启动定位）',variable=self.only_keys).pack(anchor='w',padx=10)
        self.position=tk.StringVar(value='位置：等待定位/里程计')
        ttk.Label(root,textvariable=self.position).pack(fill='x',padx=10)
        self.live = tk.StringVar(value='串口状态：尚未启动')
        ttk.Label(root,textvariable=self.live,wraplength=920).pack(fill='x',padx=10)
        self.state = tk.StringVar(value='待操作；配置目录：'+str(self.config))
        ttk.Label(root,textvariable=self.state,wraplength=920).pack(fill='x',padx=10)
        self.text = tk.Text(root,height=22,wrap='word'); self.text.pack(fill='both',expand=True,padx=10,pady=10)
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
        self.busy=True; self.state.set('处理中：'+fn.__name__)
        def worker():
            try:
                result=fn(data)
                self.events.put(('done',str(result or '操作完成')))
            except Exception as exc:self.events.put(('error',str(exc)))
        threading.Thread(target=worker,daemon=True).start()

    def poll(self):
        while not self.events.empty():
            kind,text=self.events.get()
            if kind=='map':self.values['map'].set(text)
            elif kind=='close':self.node.destroy_node();self.root.destroy();return
            else:
                self.busy=False; self.state.set(text); self.log(text)
                if kind=='error':messagebox.showerror('操作失败',text)
        rclpy.spin_once(self.node,timeout_sec=0)
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
            self.last_status=status
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
        self.real('odometry'); self.mode='mapping'; self.odom_session=None; self.rviz(True)
        # Record the first real IMU pose; ask operator to remain still until completed.
        output=run_command(['ros2','topic','echo','/Odometry','nav_msgs/msg/Odometry','--once'],45)
        (self.session_dir/'mapping_start_odom.yaml').write_text(output)
        return '建图起点已记录。现在用遥控器走场；完成后点“保存地图”。'

    def save_map(self,d):
        if self.mode!='mapping' or not self.procs.alive('sensors'):raise ValueError('请先开始建图并保持雷达运行')
        output=run_command(['ros2','run','go2_fastlio_localization','save_live_map','--ros-args',
                            '-p','output_dir:='+str(self.session_dir),'-p','timeout_sec:=45.0'],55)
        path=saved_path(output)
        self.events.put(('map',path))
        return '保存成功：'+path+'\n已自动填入定位地图栏。确认当前 IMU 地图位姿后再切到定位。'

    def prepare_localization(self,d):
        pose=[float(v) for v in d['pose'].replace(',',' ').split()]
        backup=apply_localization(self.config,d['map'],pose)
        return backup

    def localize(self,d):
        if self.procs.alive('sensors') and self.mode=='localization':
            return '定位已运行；修改地图或初始位姿后请先停止全部程序再启动。'
        self.procs.stop_all();self.mode=None
        backup=self.prepare_localization(d)
        self.real('localization');self.mode='localization';self.capture_frame='map';self.rviz(False)
        return '定位已启动，请在 RViz 核对位置后打开采点面板。配置备份：'+str(backup)

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
        self.procs.stop('bridge');self.last_status={}
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
        self.procs.stop_all();self.mode=None;self.odom_session=None
        return '本面板启动的程序已停止；未终止其他终端的程序。'

    def close(self):
        if self.busy:
            messagebox.showinfo('请稍候','当前操作结束后再关闭；地图保存最多约55秒。');return
        self.busy=True
        def worker():
            self.procs.stop_all();self.events.put(('close',''))
        threading.Thread(target=worker,daemon=True).start()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir',default=str(Path.home()/'.config/rc2026'))
    args=parser.parse_args()
    rclpy.init(args=[])
    try:
        root=tk.Tk();Panel(root,args.config_dir);root.mainloop()
    finally:
        if rclpy.ok():rclpy.shutdown()
