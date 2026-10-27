"""Map-bound startup poses and a top-view pose picker (no robot commands)."""
import hashlib
import math
from pathlib import Path
import tempfile
import os
import yaml
from .profile import vector


def pose_vector(pose):
    """Convert a ROS pose dictionary to IMU xyz/rpy, normalizing quaternion."""
    p=pose['position']; q=pose['orientation']
    xyz=vector([p[k] for k in ('x','y','z')],3,'位置')
    x,y,z,w=vector([q[k] for k in ('x','y','z','w')],4,'四元数')
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if norm<1e-8:raise ValueError('位姿四元数无效')
    x,y,z,w=(v/norm for v in (x,y,z,w))
    return xyz+[math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                math.asin(max(-1,min(1,2*(w*y-z*x)))),
                math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]


def fingerprint(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def write_start(map_path, pose, frame):
    path=Path(map_path).resolve()
    data=dict(version=1,map_sha256=fingerprint(path),pose=vector(pose,6,'建图起点'),source_frame=frame)
    target=Path(str(path)+'.start.yaml')
    with tempfile.NamedTemporaryFile('w',dir=path.parent,delete=False,encoding='utf-8') as stream:
        temp=Path(stream.name);yaml.safe_dump(data,stream,allow_unicode=True)
    try:os.replace(temp,target)
    finally:temp.unlink(missing_ok=True)
    return target


def read_start(map_path):
    path=Path(map_path).expanduser().resolve();sidecar=Path(str(path)+'.start.yaml')
    if not sidecar.exists():raise ValueError('这张地图没有建图起点记录。请选择“图上指定位置与朝向”，或手动填写位姿。')
    data=yaml.safe_load(sidecar.read_text())
    if data.get('version')!=1 or data.get('map_sha256')!=fingerprint(path):
        raise ValueError('起点记录与所选 PCD 不匹配，请重新指定位姿。')
    return vector(data['pose'],6,'建图起点')


def prepare_cloud(path):
    import numpy as np
    from go2_fastlio_localization.pcd_to_nav2_map import read_pcd_xyz
    try:xyz=read_pcd_xyz(str(Path(path).expanduser()))
    except Exception as exc:
        raise ValueError('地图预览读取失败（支持 ASCII / binary PCD）；压缩 PCD 请先转换。详情：'+str(exc)) from exc
    xyz=xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz)<2:raise ValueError('地图有效点不足，无法显示俯视图')
    # Bound Tk canvas work, while retaining the map extent.
    limits=(xyz[:,:2].min(axis=0),xyz[:,:2].max(axis=0))
    return xyz[::max(1,math.ceil(len(xyz)/7000))],limits


class PosePicker:
    def __init__(self,parent,cloud,initial,apply):
        import tkinter as tk
        from tkinter import ttk,messagebox
        self.tk=tk;self.messagebox=messagebox;self.apply=apply
        self.xyz,self.limits=cloud;self.point=None;self.heading=None
        self.root=tk.Toplevel(parent);self.root.title('旧地图：点击位置，拖动箭头指定朝向');self.root.geometry('960x760')
        ttk.Label(self.root,text='左键按下选择 IMU 的地图位置，拖动后松开指定机头方向。+X 向右，+Y 向上。',padding=10).pack(fill='x')
        self.canvas=tk.Canvas(self.root,bg='#15283b',highlightthickness=0);self.canvas.pack(fill='both',expand=True,padx=12)
        self.canvas.bind('<Configure>',lambda e:self.draw())
        self.canvas.bind('<ButtonPress-1>',self.press);self.canvas.bind('<B1-Motion>',self.drag);self.canvas.bind('<ButtonRelease-1>',self.drag)
        row=ttk.Frame(self.root,padding=10);row.pack(fill='x');self.fields=[]
        for i,name in enumerate(('z（米）','roll（弧度）','pitch（弧度）')):
            ttk.Label(row,text=name).pack(side='left')
            value=initial[i+2] if initial else ('' if i==0 else 0)
            var=tk.StringVar(value=str(value));self.fields.append(var)
            ttk.Entry(row,textvariable=var,width=10).pack(side='left',padx=6)
        self.info=tk.StringVar(value='等待选择位置与朝向')
        ttk.Label(self.root,textvariable=self.info,padding=8).pack(fill='x')
        ttk.Label(self.root,text='z 是 IMU 在旧地图中的高度，不是离地高度。roll / pitch 默认 0 仅适合近似水平摆放。',padding=8).pack(fill='x')
        ttk.Button(self.root,text='填入初始位姿（随后在主面板点尝试定位）',command=self.accept).pack(pady=10)
        if initial:self.point=initial[:2];self.heading=initial[5]

    def transform(self):
        lo,hi=self.limits;w=max(self.canvas.winfo_width(),100);h=max(self.canvas.winfo_height(),100)
        scale=min((w-40)/max(hi[0]-lo[0],.1),(h-40)/max(hi[1]-lo[1],.1))
        return (lo+hi)/2,scale,w,h

    def screen(self,x,y):
        center,s,w,h=self.transform();return w/2+(x-center[0])*s,h/2-(y-center[1])*s

    def world(self,x,y):
        center,s,w,h=self.transform();return [center[0]+(x-w/2)/s,center[1]-(y-h/2)/s]

    def draw(self):
        self.canvas.delete('all')
        for x,y,z in self.xyz:
            sx,sy=self.screen(x,y);self.canvas.create_rectangle(sx,sy,sx+1,sy+1,fill='#a2bbce',outline='')
        self.marker()

    def marker(self):
        self.canvas.delete('pose')
        if self.point is None:return
        x,y=self.screen(*self.point)
        self.canvas.create_oval(x-5,y-5,x+5,y+5,fill='#ffc45b',tags='pose')
        if self.heading is not None:
            self.canvas.create_line(x,y,x+55*math.cos(self.heading),y-55*math.sin(self.heading),fill='#ffc45b',width=3,arrow='last',tags='pose')
            self.info.set(f'x={self.point[0]:.3f} m   y={self.point[1]:.3f} m   朝向={math.degrees(self.heading):.1f}°')

    def press(self,event):
        self.point=self.world(event.x,event.y);self.heading=None;self.info.set('请拖动箭头确定朝向');self.marker()

    def drag(self,event):
        if self.point is None:return
        sx,sy=self.screen(*self.point)
        if math.hypot(event.x-sx,event.y-sy)<8:return
        self.heading=math.atan2(sy-event.y,event.x-sx);self.marker()

    def accept(self):
        try:
            if self.point is None or self.heading is None:raise ValueError('请先点击位置并拖动箭头确定朝向')
            pose=vector([*self.point,*[float(v.get()) for v in self.fields],self.heading],6,'初始位姿')
            self.apply(pose);self.root.destroy()
        except ValueError as exc:self.messagebox.showerror('位姿未填写完整',str(exc),parent=self.root)
