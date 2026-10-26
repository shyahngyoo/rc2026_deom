"""Non-GUI helpers for the onboard operator panel."""
from datetime import datetime
from pathlib import Path
import os
import shutil
import signal
import subprocess
import tempfile
import yaml
from .profile import load_profile, pcd_map, vector


def apply_localization(folder, map_path, pose):
    folder = Path(folder).expanduser().resolve()
    map_path = pcd_map(str(Path(map_path).expanduser().resolve()))
    pose = vector(pose, 6, 'current IMU pose')
    with tempfile.TemporaryDirectory(dir=folder.parent) as temp:
        candidate = Path(temp)/'config'
        shutil.copytree(folder, candidate)
        profile = yaml.safe_load((candidate/'profile.yaml').read_text())
        profile.update(map_pcd=map_path, initial_pose=pose)
        gicp = yaml.safe_load((candidate/'gicp.yaml').read_text())
        gicp['gicp_localizer']['ros__parameters'].update(initial_pose_enabled=True, initial_pose=pose)
        for name, data in [('profile.yaml', profile), ('gicp.yaml', gicp)]:
            (candidate/name).write_text(yaml.safe_dump(data, sort_keys=False))
        load_profile(candidate, 'localization')
        backup = folder.with_name(folder.name+'.backup.'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copytree(folder, backup)
        try:
            for name in ('profile.yaml', 'gicp.yaml'):
                os.replace(candidate/name, folder/name)
        except OSError:
            for name in ('profile.yaml', 'gicp.yaml'):
                shutil.copy2(backup/name, folder/name)
            raise
    return backup


class Processes:
    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.items = {}

    def alive(self, name):
        return name in self.items and self.items[name][0].poll() is None

    def start(self, name, args):
        if self.alive(name):
            raise ValueError(name+' 已在运行')
        self.stop(name)
        log = (self.log_dir/(name+'.log')).open('a')
        try:
            p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except Exception:
            log.close()
            raise
        self.items[name] = (p, log)

    def stop(self, name):
        item = self.items.pop(name, None)
        if not item:
            return
        p, log = item
        # Own process groups only; never kill unrelated ROS sessions.
        for sig, delay in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            try:
                p.wait(timeout=delay)
                # Also reap children left after their parent exits.
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
        log.close()

    def stop_all(self):
        for name in list(reversed(self.items)):
            self.stop(name)


def run_command(args, timeout=60):
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, start_new_session=True)
    try:
        output, _ = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.communicate()
        raise RuntimeError('命令超时：'+' '.join(args))
    if p.returncode:
        raise RuntimeError(output[-4000:])
    return output


def saved_path(output):
    paths = [line[6:].strip() for line in output.splitlines() if line.startswith('SAVED ')]
    if len(paths) != 1:
        raise ValueError('保存程序未返回唯一 SAVED 路径：'+output[-1500:])
    return pcd_map(paths[0])
