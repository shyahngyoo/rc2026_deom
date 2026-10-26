"""Desktop course editor: obstacle instances, TF capture and task generation."""
import copy
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
import yaml
from ament_index_python.packages import get_package_share_directory
from quadruped_stm32_bridge.course import new_instance, compile_course, write_yaml, pose3


class CoursePanel:
    def __init__(self,parent):
        self.parent=parent
        self.path=parent.config/'cybergear_course.yaml'
        default=Path(get_package_share_directory('quadruped_stm32_bridge'))/'config/cybergear_obstacles.yaml'
        self.templates=yaml.safe_load(default.read_text())['templates']
        self.course=dict(version=1,frame=parent.capture_frame,session=parent.odom_session,map_pcd=parent.values['map'].get(),obstacles=[])
        if self.path.exists():self.course=yaml.safe_load(self.path.read_text())
        self.root=tk.Toplevel(parent.root);self.root.title('赛场配置：选障碍 → 采点 → 运行');self.root.geometry('1100x780')
        self.selected=None
        top=ttk.Frame(self.root,padding=8);top.pack(fill='x')
        self.choices={k+' / '+v['name']:k for k,v in self.templates.items()}
        self.choice=tk.StringVar(value=next(iter(self.choices)))
        ttk.Combobox(top,textvariable=self.choice,values=list(self.choices),state='readonly',width=18).pack(side='left')
        for label,fn in [('新场地',self.new),('添加障碍',self.add),('打开场地',self.load),('另存场地',self.save_as),('保存',self.save)]:
            ttk.Button(top,text=label,command=lambda f=fn:self.guard(f)).pack(side='left',padx=3)
        self.frame_text=tk.StringVar();ttk.Label(self.root,textvariable=self.frame_text).pack(anchor='w',padx=8)
        body=ttk.Panedwindow(self.root,orient='horizontal');body.pack(fill='both',expand=True,padx=8)
        left=ttk.Frame(body);right=ttk.Frame(body);body.add(left,weight=1);body.add(right,weight=3)
        self.list=tk.Listbox(left,width=30,exportselection=False);self.list.pack(fill='both',expand=True)
        self.list.bind('<<ListboxSelect>>',self.select)
        for label,fn in [('上移',lambda:self.move(-1)),('下移',lambda:self.move(1)),('启用/跳过',self.toggle),('删除',self.remove)]:
            ttk.Button(left,text=label,command=lambda f=fn:self.guard(f)).pack(fill='x')
        namebar=ttk.Frame(right);namebar.pack(fill='x')
        ttk.Label(namebar,text='障碍名称').pack(side='left');self.name=tk.StringVar()
        ttk.Entry(namebar,textvariable=self.name).pack(side='left',fill='x',expand=True)
        tuning=ttk.Frame(right);tuning.pack(fill='x');self.tuning={}
        for key,label in [('x_scale','X距离倍率'),('y_scale','Y距离倍率'),('jump_wait_scale','动作等待倍率')]:
            ttk.Label(tuning,text=label).pack(side='left');self.tuning[key]=tk.StringVar(value='1.0')
            ttk.Entry(tuning,textvariable=self.tuning[key],width=6).pack(side='left')
        self.note=tk.StringVar();ttk.Label(right,textvariable=self.note,wraplength=700).pack(fill='x')
        samples=ttk.Frame(right);samples.pack(fill='x')
        for label,key in [('采原点＋模板X方向','origin'),('采入口＋朝向','entry'),('采出口＋朝向','exit')]:
            ttk.Button(samples,text=label,command=lambda k=key:self.guard(lambda:self.capture(k))).pack(side='left',padx=2)
        self.coords=tk.StringVar();ttk.Label(right,textvariable=self.coords,wraplength=700).pack(fill='x')
        actions=ttk.Frame(right);actions.pack(fill='x')
        for label,fn in [('添加入口前过渡点',self.transit),('清空过渡点',self.clear_transit),('导入旧 field 采点',self.import_field)]:
            ttk.Button(actions,text=label,command=lambda f=fn:self.guard(f)).pack(side='left',padx=2)
        ttk.Label(right,text='模板已填好。要调距离/转角/等待可编辑下方；target 为局部 x/y/角度(度)，null 保持当前分量。').pack(fill='x')
        self.editor=tk.Text(right,wrap='none',height=24);self.editor.pack(fill='both',expand=True)
        bottom=ttk.Frame(self.root,padding=8);bottom.pack(fill='x')
        for label,fn in [('生成并载入当前障碍',lambda:self.prepare(True)),('生成并载入启用路线',lambda:self.prepare(False)),
                         ('开始 / 从头重跑',lambda:parent.dispatch(parent.start_task)),('停止任务',lambda:parent.dispatch(parent.stop_task))]:
            ttk.Button(bottom,text=label,command=lambda f=fn:self.guard(f)).pack(side='left',padx=3)
        self.live=tk.StringVar(value='载入后在状态就绪时点开始；不会自动运动。')
        ttk.Label(self.root,textvariable=self.live,wraplength=1000).pack(fill='x',padx=8)
        self.refresh();self.root.after(300,self.update_status)
        self.root.protocol('WM_DELETE_WINDOW',self.close)

    def guard(self,fn):
        try:fn()
        except Exception as exc:messagebox.showerror('场地操作失败',str(exc),parent=self.root)

    def current(self):
        if self.selected is None:raise ValueError('请先选择或添加障碍')
        return self.course['obstacles'][self.selected]

    def flush(self):
        if self.selected is None:return
        steps=yaml.safe_load(self.editor.get('1.0','end'))
        if not isinstance(steps,list) or not steps:raise ValueError('模板步骤必须是非空列表')
        item=self.current();item['steps']=steps
        from quadruped_stm32_bridge.cybergear_core import number
        for key,value in self.tuning.items():item[key]=number(float(value.get()),key,True)
        if not self.name.get().strip():raise ValueError('名称不能为空')
        item['name']=self.name.get().strip()

    def refresh(self):
        self.list.delete(0,'end')
        for item in self.course['obstacles']:
            self.list.insert('end',('✓ ' if item.get('enabled',True) else '跳过 ')+item['name'])
        self.frame_text.set('坐标系：'+self.course['frame']+'　保存文件：'+str(self.path))
        if self.selected is not None:
            self.list.selection_set(self.selected);self.show()

    def show(self):
        item=self.current();self.name.set(item['name'])
        for key,value in self.tuning.items():value.set(str(item.get(key,1.0)))
        self.note.set(self.templates.get(item['template'],{}).get('note',''))
        self.coords.set('已采点：'+str(item['samples'])+'　过渡点数：'+str(len(item.get('transit',[]))))
        self.editor.delete('1.0','end');self.editor.insert('1.0',yaml.safe_dump(item['steps'],allow_unicode=True,sort_keys=False))

    def select(self,event):
        if not self.list.curselection():return
        index=self.list.curselection()[0]
        try:self.flush()
        except Exception as exc:
            messagebox.showerror('先修正当前模板',str(exc),parent=self.root);return
        self.selected=index;self.show()

    def new(self):
        self.save()
        path=filedialog.asksaveasfilename(parent=self.root,defaultextension='.yaml',title='为新场地选择独立文件名')
        if not path:return
        if Path(path).exists() and not messagebox.askyesno('覆盖','这个文件已存在，覆盖为新场地？',parent=self.root):return
        self.course=dict(version=1,frame=self.parent.capture_frame,session=self.parent.odom_session,map_pcd=self.parent.values['map'].get(),obstacles=[])
        self.path=Path(path);self.selected=None;self.editor.delete('1.0','end');self.refresh();self.save()

    def add(self):
        self.flush();key=self.choices[self.choice.get()]
        ids={i['id'] for i in self.course['obstacles']};n=1
        while key+'_'+str(n) in ids:n+=1
        self.course['obstacles'].append(new_instance(key,self.templates[key],key+'_'+str(n)))
        self.selected=len(self.course['obstacles'])-1;self.refresh();self.save()

    def move(self,delta):
        self.flush();self.current();i=self.selected;j=i+delta
        if 0<=j<len(self.course['obstacles']):
            items=self.course['obstacles'];items[i],items[j]=items[j],items[i];self.selected=j
        self.refresh();self.save()

    def toggle(self):
        self.flush();i=self.current();i['enabled']=not i.get('enabled',True);self.refresh();self.save()

    def remove(self):
        self.current()
        if not messagebox.askyesno('删除障碍','删除这个障碍实例及其采点？',parent=self.root):return
        self.course['obstacles'].pop(self.selected);self.selected=None;self.editor.delete('1.0','end');self.refresh();self.save()

    def fresh_pose(self):
        if self.parent.last_status.get('task')=='running':raise ValueError('先停止自动任务再采点')
        if self.course['frame']!=self.parent.capture_frame:raise ValueError('场地坐标系与当前定位模式不同，请打开对应场地文件')
        if self.course['frame']=='nav_odom' and self.course.get('session')!=self.parent.odom_session:
            if any(i['samples'] or i.get('transit') for i in self.course['obstacles']):
                if not messagebox.askyesno('里程计已重启','旧点不再对齐。清空本场所有采点并在当前里程计重采？',parent=self.root):raise ValueError('未更改旧采点')
                for i in self.course['obstacles']:i['samples']={};i['transit']=[]
            self.course['session']=self.parent.odom_session
        return self.parent.read_capture_pose()

    def capture(self,key):
        self.flush();item=self.current();pose=self.fresh_pose();item['samples'][key]=pose;self.save();self.show()

    def transit(self):
        self.flush();self.current();pose=self.fresh_pose();self.current().setdefault('transit',[]).append(pose);self.save();self.show()

    def clear_transit(self):
        self.current()['transit']=[];self.save();self.show()

    def import_field(self):
        self.flush();item=self.current()
        if self.course['frame']!='map':raise ValueError('旧 field.yaml 的 map 坐标不能导入实时里程计场地')
        path=filedialog.askopenfilename(parent=self.root,filetypes=[('YAML','*.yaml')])
        if not path:return
        data=yaml.safe_load(Path(path).read_text());samples=data['samples']
        prefix=simpledialog.askstring('旧采点标签','标签前缀，例如 bridge_a',initialvalue=item['template'],parent=self.root)
        if not prefix:return
        # Legacy begin yaw is not the template axis. Derive axis from start/finish.
        start=pose3(samples['start']);finish=pose3(samples['finish'])
        import math
        if math.hypot(finish[0]-start[0],finish[1]-start[1])<0.05:raise ValueError('start/finish 太近，无法确定模板轴')
        origin=pose3(samples[prefix+'.begin']);origin[2]=math.atan2(finish[1]-start[1],finish[0]-start[0])
        captured=dict(origin=origin,entry=pose3(samples[prefix+'.entry']),exit=pose3(samples[prefix+'.exit']))
        item['samples']=captured;self.save();self.show()

    def save(self):
        self.flush();write_yaml(self.path,self.course)

    def save_as(self):
        path=filedialog.asksaveasfilename(parent=self.root,defaultextension='.yaml')
        if path:self.path=Path(path);self.save();self.refresh()

    def load(self):
        path=filedialog.askopenfilename(parent=self.root,filetypes=[('YAML','*.yaml')])
        if not path:return
        data=yaml.safe_load(Path(path).read_text())
        if not isinstance(data,dict) or data.get('frame') not in ('map','nav_odom') or not isinstance(data.get('obstacles'),list):raise ValueError('不是 CyberGear 场地文件')
        self.course=data;self.path=Path(path);self.selected=None;self.editor.delete('1.0','end');self.refresh()

    def prepare(self,single):
        self.save()
        if self.course['frame']=='nav_odom' and self.course.get('session')!=self.parent.odom_session:raise ValueError('里程计会话已改变，请重采本场点位')
        if self.course['frame']=='map' and self.course.get('map_pcd') and str(Path(self.course['map_pcd']).expanduser().resolve())!=str(Path(self.parent.values['map'].get()).expanduser().resolve()):raise ValueError('本场采点地图与主面板所选地图不同，请先选回对应地图或建立新场地')
        result=compile_course(self.course,self.current()['id'] if single else None)
        path=self.parent.config/('cybergear_selected.yaml' if single else 'cybergear_route.yaml')
        write_yaml(path,result)
        self.parent.values['plan'].set(str(path));self.parent.only_keys.set(False)
        self.parent.dispatch(self.parent.automatic)
        self.live.set('任务已生成。等待主面板就绪，然后点开始；单障碍会先走到入口。')

    def update_status(self):
        if self.root.winfo_exists():
            self.live.set(self.parent.live.get());self.root.after(300,self.update_status)

    def close(self):
        try:self.save()
        except Exception as exc:
            if not messagebox.askyesno('未保存',str(exc)+'\n仍关闭？',parent=self.root):return
        self.root.destroy()
