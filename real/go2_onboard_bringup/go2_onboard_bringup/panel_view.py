"""Tk presentation and pure status formatting for the onboard console."""
import tkinter as tk
from tkinter import ttk, font
from pathlib import Path

MODE_CHOICES = {'普通运动 · 9': 9, '小坡 · 10': 10, '木桥 B · 11': 11,
                '木桥 A · 12': 12, '大斜坡 · 13': 13, '爬行 / 限高 · 14': 14,
                '沙坑 · 15': 15, '绕杆 · 16': 16, '高墙 · 17': 17, '台阶 · 18': 18}
MODE_NAMES = dict(zip('acdefghjkb', ['普通运动','小坡','木桥 B','木桥 A','大斜坡','爬行 / 限高','沙坑','绕杆','高墙','台阶']))


def status_summary(status, bridge_alive, age):
    if not bridge_alive:
        return '串口未启动', '到「单动作调试」连接串口，或载入自动任务。', '等待回包'
    if age is None:
        return '等待状态', '串口程序已启动，尚未收到状态消息；检查运行日志。', '等待回包'
    if age > 2:
        return '状态已过期', '串口程序没有继续更新状态；检查运行日志。', '回包已过期'
    if not status.get('connected'):
        return '串口未连接', '检查设备路径、dialout 权限，以及串口是否被其他程序占用。', '等待连接'
    feedback = status.get('status') or {}
    mode = feedback.get('mode')
    mode_text = MODE_NAMES.get(mode, '未初始化' if mode == 'z' else str(mode or '等待回包'))
    if not feedback:
        return '等待 STM32', '完成机器人初始化，将遥控器从手动切到自动；检查接线和波特率。', mode_text
    if mode == 'z':
        return '尚未初始化', '先按电控流程完成机器人初始化。', mode_text
    if feedback.get('fault', 0):
        return '电控故障', f"故障 {feedback['fault']} · 电机 {feedback.get('motor_id')} · 详情 {feedback.get('detail')}", mode_text
    if not status.get('ready'):
        return '回包未就绪', f"检查回包是否超时；数据年龄 {status.get('status_age_s')} 秒。", mode_text
    if status.get('pending_mode'):
        return '等待模式确认', '已发送模式键，等待 STM32 返回对应模式。', mode_text
    return '就绪：True', '通信状态正常；可调单动作，或准备好定位后开始任务。', mode_text


def build_view(p, profile):
    root = p.root
    root.title('RC2026 · 机器人操作台')
    root.geometry('1180x880'); root.minsize(980, 740)
    root.configure(bg='#edf2f7')
    families=set(font.families(root))
    family=next((name for name in ('Microsoft YaHei','Noto Sans CJK SC','Droid Sans Fallback') if name in families),'sans-serif')
    style = ttk.Style(root); style.theme_use('clam')
    style.configure('.', font=(family, 11), background='#edf2f7', foreground='#20344a')
    style.configure('TButton', padding=(14, 7), background='#e2eaf2', borderwidth=0)
    style.map('TButton', background=[('active','#cbdceb')])
    style.configure('Accent.TButton', background='#176b98', foreground='white')
    style.map('Accent.TButton', background=[('active','#125579')])
    style.configure('Stop.TButton', background='#b83e48', foreground='white')
    style.map('Stop.TButton', background=[('active','#922e37')])
    style.configure('TNotebook.Tab', padding=(20,12))
    style.map('TNotebook.Tab', background=[('selected','#176b98')], foreground=[('selected','white')])
    style.configure('TLabelframe', padding=12)
    style.configure('TLabelframe.Label', font=(family,12,'bold'))
    header=tk.Frame(root,bg='#172d43');header.pack(fill='x')
    tk.Label(header,text='RC2026  /  机器人操作台',font=(family,21,'bold'),fg='white',bg='#172d43').pack(side='left',padx=20,pady=16)
    ttk.Button(header,text='停止任务',style='Stop.TButton',command=lambda:p.dispatch(p.stop_task)).pack(side='right',padx=20)
    cards=ttk.Frame(root,padding=(16,12));cards.pack(fill='x')
    p.connection=tk.StringVar(value='串口未启动');p.robot_mode=tk.StringVar(value='等待回包')
    p.task_state=tk.StringVar(value='未载入任务')
    for i,(title,var) in enumerate([('通信',p.connection),('STM32 当前模式',p.robot_mode),('自动任务',p.task_state)]):
        c=tk.Frame(cards,bg='white',padx=16,pady=10);c.grid(row=0,column=i,sticky='nsew',padx=4);cards.columnconfigure(i,weight=1)
        tk.Label(c,text=title,bg='white',fg='#60778b',font=(family,10)).pack(anchor='w')
        tk.Label(c,textvariable=var,bg='white',fg='#176b98',font=(family,16,'bold')).pack(anchor='w',pady=(4,0))
    p.status_hint=tk.StringVar(value='先选择建图 / 定位，或直接连接串口调试。')
    ttk.Label(root,textvariable=p.status_hint,wraplength=940).pack(fill='x',padx=24)
    p.position=tk.StringVar(value='位置：等待定位 / 里程计')
    ttk.Label(root,textvariable=p.position).pack(fill='x',padx=24,pady=(5,10))
    p.values={}
    defaults={'map_name':'field','map':profile.get('map_pcd') or '',
              'pose':' '.join(map(str,profile.get('initial_pose') or [])),
              'port':'/dev/ttyUSB0','plan':str(p.config/'cybergear_plan.yaml'),
              'field':str(Path.home()/'rc2026_field/field.yaml')}
    for k,v in defaults.items():p.values[k]=tk.StringVar(value=v)
    p.only_keys=tk.BooleanVar(value=False)
    tabs=ttk.Notebook(root);tabs.pack(fill='both',expand=True,padx=20,pady=10)
    def page(title,description):
        frame=ttk.Frame(tabs,padding=18);tabs.add(frame,text=title)
        ttk.Label(frame,text=description,wraplength=900).pack(anchor='w',pady=(0,14))
        return frame
    def field(parent,key,label,browse=False):
        row=ttk.Frame(parent);row.pack(fill='x',pady=5)
        ttk.Label(row,text=label,width=18).pack(side='left')
        ttk.Entry(row,textvariable=p.values[key]).pack(side='left',fill='x',expand=True)
        if browse:ttk.Button(row,text='选择文件',command=lambda:p.choose(key)).pack(side='left',padx=(8,0))
    def buttons(parent,items):
        row=ttk.Frame(parent);row.pack(fill='x',pady=10)
        for label,fn,accent in items:
            ttk.Button(row,text=label,style=accent or 'TButton',command=lambda f=fn:p.dispatch(f)).pack(side='left',padx=(0,8))
    mapping=page('01  建图与定位','建图完成后保存 PCD；已有地图可直接定位。临时调试可使用快速里程计。')
    scroll=ttk.Scrollbar(mapping,orient='vertical');scroll.pack(side='right',fill='y')
    canvas=tk.Canvas(mapping,highlightthickness=0,bg='#edf2f7',yscrollcommand=scroll.set)
    canvas.pack(fill='both',expand=True);scroll.configure(command=canvas.yview)
    content=ttk.Frame(canvas);window=canvas.create_window(0,0,anchor='nw',window=content)
    content.bind('<Configure>',lambda e:canvas.configure(scrollregion=canvas.bbox('all')))
    canvas.bind('<Configure>',lambda e:canvas.itemconfigure(window,width=e.width))
    mapping=content
    field(mapping,'map_name','本次建图名称')
    buttons(mapping,[('开始建图',p.mapping,'Accent.TButton'),('保存地图',p.save_map,''),('停止建图 / 定位',p.stop_sensors,'')])
    ttk.Label(mapping,text='放弃本次建图：停止 → 重新开始。不会删除已保存地图；未保存数据不保证保留。').pack(anchor='w')
    field(mapping,'map','定位 PCD',True);field(mapping,'pose','IMU 初始位姿')
    ttk.Label(mapping,text='位姿依次填写 x y z roll pitch yaw，单位为米、弧度。').pack(anchor='w')
    buttons(mapping,[('使用建图起点位姿',p.use_map_start,''),('图上指定位置与朝向',p.preview_map,'')])
    buttons(mapping,[('尝试定位 / 重新匹配',p.try_localization,'Accent.TButton'),('使用定位结果',p.use_match,''),('快速里程计（无需地图）',p.quick_odometry,'')])
    p.localization_state=tk.StringVar(value='定位：尚未尝试定位')
    ttk.Label(mapping,textvariable=p.localization_state,wraplength=850).pack(fill='x',pady=5)
    ttk.Label(mapping,text='旧地图：点选位置和朝向 → 尝试定位 → 在 RViz 核对点云重合 → 使用定位结果。',wraplength=850).pack(fill='x',pady=5)

    capture=page('02  场地与采点','选择障碍 → 采原点、入口、出口 → 调整顺序 → 生成任务。')
    ttk.Button(capture,text='打开场地 / 障碍编排与采点',style='Accent.TButton',command=p.open_course).pack(anchor='w',pady=10)
    for tip in ['原点：定义模板坐标零点与 +X 方向，不能任意当作入口。','入口：自动接近障碍时的位置与朝向。出口：越障后的目标位置与朝向。','过渡点：按顺序引导绕行；此模式不会自动避障。','快速里程计重启后需要重采；相同地图定位下可复用已保存采点。']:
        ttk.Label(capture,text=tip,wraplength=880).pack(anchor='w',pady=7)
    legacy=ttk.LabelFrame(capture,text='兼容旧采点流程');legacy.pack(fill='x',pady=14)
    field(legacy,'field','旧采点文件');buttons(legacy,[('打开旧采点面板',p.capture,'')])
    debug=page('03  单动作调试','完成电控初始化 → 遥控器切自动 → 等待就绪 → 发送模式 → 测试动作。')
    field(debug,'port','串口设备')
    buttons(debug,[('连接串口单动作调试',p.debug_bridge,'Accent.TButton')])
    row=ttk.Frame(debug);row.pack(fill='x',pady=8)
    p.debug_key=tk.StringVar(value=next(iter(MODE_CHOICES)))
    ttk.Combobox(row,textvariable=p.debug_key,values=list(MODE_CHOICES),state='readonly',width=24).pack(side='left')
    ttk.Button(row,text='发送模式键',command=lambda:p.manual_key(MODE_CHOICES[p.debug_key.get()])).pack(side='left',padx=10)
    pad=ttk.Frame(debug);pad.pack(anchor='w',pady=(2,0))
    for label,key,r,c in [('前进',2,0,1),('左转',1,1,0),('停止',5,1,1),('右转',3,1,2),('后退',4,2,1),('动作6',6,0,4),('动作7',7,1,4),('动作8',8,2,4)]:
        ttk.Button(pad,text=label,width=12,style='Stop.TButton' if key==5 else 'TButton',command=lambda k=key:p.manual_key(k)).grid(row=r,column=c,padx=5,pady=2)
    ttk.Label(debug,text='每次点击发送一次。前进可能持续运动，需再点停止；动作6/7/8由当前电控模式解释。',wraplength=900).pack(anchor='w')
    auto=page('04  自动运行','先在场地窗口生成并载入单障碍，调好后再生成整条路线。载入后需点击开始才运行。')
    field(auto,'plan','任务 YAML',True);field(auto,'port','串口设备')
    ttk.Checkbutton(auto,text='只调按键 / 时序（任务不包含位置航点）',variable=p.only_keys).pack(anchor='w',pady=10)
    buttons(auto,[('载入任务',p.automatic,'Accent.TButton'),('开始 / 从头重跑',p.start_task,'Accent.TButton'),('停止任务',p.stop_task,'Stop.TButton')])
    p.live=tk.StringVar(value='等待任务状态')
    ttk.Label(auto,textvariable=p.live,wraplength=900).pack(fill='x',pady=10)
    ttk.Label(auto,text='停止后再次开始会从第一步执行。修改采点或模板后，需要重新生成并载入任务。',wraplength=900).pack(anchor='w')
    logpage=page('运行日志','启动失败或状态异常时，在这里查看操作记录与程序日志。')
    p.text=tk.Text(logpage,height=10,wrap='word',bg='#15283b',fg='#dbe8f4',insertbackground='white',relief='flat',padx=12,pady=10)
    scroll=ttk.Scrollbar(logpage,command=p.text.yview);scroll.pack(side='right',fill='y')
    p.text.configure(yscrollcommand=scroll.set);p.text.pack(fill='both',expand=True)
    footer=ttk.Frame(root,padding=(20,10));footer.pack(side='bottom',fill='x',before=tabs)
    p.state=tk.StringVar(value='待操作')
    ttk.Label(footer,textvariable=p.state,wraplength=700).pack(side='left',fill='x',expand=True)
    ttk.Button(footer,text='停止本面板全部程序',command=lambda:p.dispatch(p.stop_all)).pack(side='right')
