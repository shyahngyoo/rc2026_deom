'Offline installer contracts: fake system/build tools and real local Git fixtures.'
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import pytest
import yaml

REPO=Path(__file__).resolve().parents[2]
SCRIPT=REPO/'tools/install_real.bash'
REL=('common/go2_obstacle_interfaces','common/go2_fastlio_localization','common/go2_nav_bringup',
 'common/robocon_mission','common/go2_mission_backends','real/go2_real_localization',
 'real/go2_real_bringup','real/go2_onboard_bringup')
PATCHES=('livox_driver2_monotonic_clock.patch','fast_lio_handheld.patch','fast_lio_timestamp_guard.patch')


def run(script,workspace,env=None,*flags):
    return subprocess.run(['bash',str(script),'--workspace',str(workspace),*flags],env=env,
        text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=25.)


def executable(path,content):
    path.write_text(content);path.chmod(0o755)


@pytest.fixture
def sandbox(tmp_path):
    root=tmp_path/'release';(root/'tools').mkdir(parents=True);(root/'patches').mkdir()
    shutil.copy2(SCRIPT,root/'tools/install_real.bash')
    shutil.copy2(REPO/'dependencies.repos',root/'dependencies.repos')
    for relative in REL:
        p=root/relative;p.mkdir(parents=True)
        (p/'package.xml').write_text('<package format="3"><name>'+p.name+'</name></package>')
    tiny={PATCHES[0]:('clock.txt','before\n','after\n'),
          PATCHES[1]:('map.txt','before\n','after\n'),PATCHES[2]:('time.txt','before\n','after\n')}
    lock=yaml.safe_load((REPO/'dependencies.lock.yaml').read_text())
    for patch,(file,old,new) in tiny.items():
        data='diff --git a/'+file+' b/'+file+'\n--- a/'+file+'\n+++ b/'+file+'\n@@ -1 +1 @@\n-'+old+'+'+new
        (root/'patches'/patch).write_text(data)
        lock['patches'][patch]=hashlib.sha256(data.encode()).hexdigest()
    (root/'dependencies.lock.yaml').write_text(yaml.safe_dump(lock,sort_keys=False))
    fixtures=tmp_path/'fixtures';fixtures.mkdir();urls={}
    for name,source in yaml.safe_load((root/'dependencies.repos').read_text())['repositories'].items():
        target=fixtures/name;target.mkdir()
        for file in ('clock.txt','map.txt','time.txt'):(target/file).write_text('before\n')
        (target/'CMakeLists.txt').write_text('target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")\n')
        (target/'package_ROS2.xml').write_text('<package format="3"><name>livox_ros_driver2</name></package>\n')
        subprocess.run(['/usr/bin/git','init','-q',str(target)],check=True)
        subprocess.run(['/usr/bin/git','-C',str(target),'add','.'],check=True)
        subprocess.run(['/usr/bin/git','-C',str(target),'-c','user.name=fixture','-c','user.email=fixture@localhost',
                        'commit','-qm','fixture'],check=True)
        urls[source['url']]={'path':str(target),'pin':source['version']}
    bin=tmp_path/'bin';bin.mkdir();log=tmp_path/'tools.jsonl'
    logger='''import os,json,sys
with open(os.environ['FAKE_TOOL_LOG'],'a') as f:f.write(json.dumps({'tool':os.path.basename(sys.argv[0]),'args':sys.argv[1:],'makeflags':os.environ.get('MAKEFLAGS'),'ament':os.environ.get('AMENT_PREFIX_PATH','')})+'\\n')
'''
    executable(bin/'git','#!/usr/bin/python3\n'+logger+'''import subprocess
args=sys.argv[1:]
while args and args[0]=='-c':args=args[2:]
if args[0]=='clone':
    source=json.loads(os.environ['FAKE_DEPENDENCIES'])[args[-2]]
    clone=args[:];clone[-2]=source['path']
    subprocess.run(['/usr/bin/git',*clone],check=True)
    subprocess.run(['/usr/bin/git','-C',args[-1],'config','rc2026.fakepin',source['pin']],check=True)
    sys.exit(0)
if len(args)>3 and args[0]=='-C' and args[2:]==['rev-parse','HEAD']:
    found=subprocess.run(['/usr/bin/git','-C',args[1],'config','--get','rc2026.fakepin'],capture_output=True,text=True)
    if found.returncode==0:print(found.stdout.strip());sys.exit(0)
if len(args)>4 and args[0]=='-C' and args[2:4]==['checkout','--detach']:
    real=subprocess.check_output(['/usr/bin/git','-C',args[1],'rev-parse','HEAD'],text=True).strip()
    args=args[:4]+[real]
sys.exit(subprocess.run(['/usr/bin/git',*args]).returncode)
''')
    executable(bin/'sudo','#!/usr/bin/python3\n'+logger+'''os.execvp(sys.argv[1],sys.argv[1:])
''')
    executable(bin/'dpkg','#!/usr/bin/python3\n'+'''import os,sys
if sys.argv[1:]==['--print-architecture']:print(os.environ.get('FAKE_ARCH','amd64'))
else:raise SystemExit('unexpected real dpkg operation in test')
''')
    for name in ('apt-get','rosdep','add-apt-repository','locale-gen','curl'):
        executable(bin/name,'#!/usr/bin/python3\n'+logger)
    executable(bin/'cmake','#!/usr/bin/python3\n'+logger+'''from pathlib import Path
args=sys.argv[1:]
if '-S' in args:
    build=Path(args[args.index('-B')+1]);build.mkdir(parents=True,exist_ok=True)
    prefix=[a.split('=',1)[1] for a in args if a.startswith('-DCMAKE_INSTALL_PREFIX=')][0]
    (build/'sdk_prefix').write_text(prefix)
elif '--install' in args:
    prefix=Path((Path(args[args.index('--install')+1])/'sdk_prefix').read_text())
    (prefix/'lib').mkdir(parents=True,exist_ok=True);(prefix/'include').mkdir(exist_ok=True)
    (prefix/'lib/liblivox_lidar_sdk_shared.so').write_bytes(b'fixture')
    (prefix/'include/livox_lidar_api.h').write_text('//fixture')
''')
    executable(bin/'colcon','#!/usr/bin/python3\n'+logger+'''from pathlib import Path
args=sys.argv[1:];prefix=Path(args[args.index('--install-base')+1]);prefix.mkdir(parents=True,exist_ok=True)
(prefix/'setup.bash').write_text('# fake offline build only\\n')
''')
    env=os.environ.copy();env.update(PATH=str(bin)+':'+env['PATH'],FAKE_TOOL_LOG=str(log),
        FAKE_DEPENDENCIES=json.dumps(urls),AMENT_PREFIX_PATH='/must_not_use_original_ws/install')
    return root,tmp_path/'workspace',env,log


def events(log):
    return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []


def test_bash_syntax_and_locked_runtime_sources():
    subprocess.run(['bash','-n',str(SCRIPT)],check=True)
    data=yaml.safe_load((REPO/'dependencies.repos').read_text())['repositories']
    assert set(data)=={'Livox-SDK2','livox_ros_driver2','FAST_LIO','fast_gicp'}
    assert all(len(v['version'])==40 for v in data.values())
    lock=yaml.safe_load((REPO/'dependencies.lock.yaml').read_text())
    for name,digest in lock['patches'].items():assert hashlib.sha256((REPO/'patches'/name).read_bytes()).hexdigest()==digest


def test_production_dry_run_has_no_filesystem_side_effect(tmp_path):
    ws=tmp_path/'untouched'
    r=run(SCRIPT,ws,None,'--dry-run');assert r.returncode==0,r.stdout
    assert not ws.exists()
    assert '--base-paths' in r.stdout and '--executor sequential' in r.stdout
    assert '-DBUILD_VGICP_CUDA=OFF' in r.stdout and 'configure_robot --help' in r.stdout
    commands=[line for line in r.stdout.splitlines() if line.startswith('+ ')]
    assert not any('gazebo' in c.lower() or 'champ' in c.lower() for c in commands)
    assert not any('rosdep install --from-paths src' in c or '--recursive https://github.com/koide' in c for c in commands)


def test_fake_install_is_idempotent_and_builds_only_runtime(sandbox):
    root,ws,env,log=sandbox
    first=run(root/'tools/install_real.bash',ws,env);assert first.returncode==0,first.stdout
    records=events(log);clones=[e for e in records if e['tool']=='git' and 'clone' in e['args']]
    assert len(clones)==4
    assert (ws/'src/rc2026_deom').resolve()==root
    assert (ws/'src/livox_ros_driver2/package.xml').read_bytes()==(ws/'src/livox_ros_driver2/package_ROS2.xml').read_bytes()
    cmake=(ws/'src/livox_ros_driver2/CMakeLists.txt').read_text()
    assert cmake.count('rosidl_generator_cpp')==1
    colcon=[e for e in records if e['tool']=='colcon'];assert len(colcon)==1
    assert colcon[0]['args'][:3]==['--log-base',str(ws/'log'),'build']
    assert colcon[0]['makeflags']=='-j2' and 'original_ws' not in colcon[0]['ament']
    assert '--executor' in colcon[0]['args'] and 'sequential' in colcon[0]['args']
    dep_install=[e for e in records if e['tool']=='rosdep' and e['args'][0]=='install'][0]['args']
    start=dep_install.index('--from-paths')+1;stop=dep_install.index('--ignore-src')
    assert dep_install[start:stop]==[str(root/p) for p in REL]+[str(ws/'src'/n) for n in ('livox_ros_driver2','FAST_LIO','fast_gicp')]
    assert {dep_install[i+1] for i,a in enumerate(dep_install) if a=='-t'}=={'build','buildtool','build_export','buildtool_export','exec'}
    assert '-DCMAKE_INSTALL_RPATH='+str(ws/'install/livox-sdk2/lib') in colcon[0]['args']
    second=run(root/'tools/install_real.bash',ws,env);assert second.returncode==0,second.stdout
    assert len([e for e in events(log) if e['tool']=='git' and 'clone' in e['args']])==4
    assert (ws/'src/livox_ros_driver2/CMakeLists.txt').read_text()==cmake
    assert all('gazebo' not in ' '.join(e['args']).lower() for e in events(log) if e['tool'] in ('apt-get','colcon','rosdep'))


def test_unknown_source_modification_is_preserved_and_rejected_before_apt(sandbox):
    root,ws,env,log=sandbox
    r=run(root/'tools/install_real.bash',ws,env);assert r.returncode==0,r.stdout
    source=ws/'src/livox_ros_driver2/clock.txt';source.write_text('user modification\n')
    before=len(events(log));r=run(root/'tools/install_real.bash',ws,env)
    assert r.returncode!=0 and 'dependency modified' in r.stdout
    assert source.read_text()=='user modification\n'
    assert not any(e['tool'] in ('sudo','apt-get','cmake','colcon') for e in events(log)[before:])


def test_untracked_unknown_file_is_not_cleaned(sandbox):
    root,ws,env,log=sandbox
    r=run(root/'tools/install_real.bash',ws,env);assert r.returncode==0,r.stdout
    file=ws/'src/fast_gicp/user_notes.txt';file.write_text('keep me')
    r=run(root/'tools/install_real.bash',ws,env)
    assert r.returncode!=0 and file.read_text()=='keep me'


def test_different_workspace_checkout_is_not_replaced(sandbox):
    root,ws,env,log=sandbox
    conflict=ws/'src/rc2026_deom';conflict.mkdir(parents=True);(conflict/'mine').write_text('untouched')
    r=run(root/'tools/install_real.bash',ws,env,'--dry-run')
    assert r.returncode!=0 and 'another checkout' in r.stdout
    assert (conflict/'mine').read_text()=='untouched' and not log.exists()


@pytest.mark.parametrize('architecture',['amd64','arm64'])
def test_supported_architecture_dry_run_is_read_only(sandbox,architecture):
    root,ws,env,log=sandbox;env['FAKE_ARCH']=architecture
    r=run(root/'tools/install_real.bash',ws,env,'--dry-run')
    assert r.returncode==0,r.stdout
    assert architecture in r.stdout and not ws.exists() and not log.exists()


def test_unsupported_architecture_and_invalid_jobs_fail_without_install(sandbox):
    root,ws,env,log=sandbox;env['FAKE_ARCH']='riscv64'
    r=run(root/'tools/install_real.bash',ws,env,'--dry-run');assert r.returncode!=0
    env['FAKE_ARCH']='amd64'
    r=run(root/'tools/install_real.bash',ws,env,'--jobs','8');assert r.returncode!=0
    assert not ws.exists() and not log.exists()


def test_patch_checksum_mismatch_is_rejected(sandbox):
    root,ws,env,log=sandbox
    with (root/'patches'/PATCHES[0]).open('a') as f:f.write('changed\n')
    r=run(root/'tools/install_real.bash',ws,env,'--dry-run')
    assert r.returncode!=0 and 'patch checksum mismatch' in r.stdout
    assert not ws.exists() and not log.exists()


def test_ros_bootstrap_requires_opt_in_and_official_source_dry_run(sandbox):
    root,ws,env,log=sandbox;p=root/'tools/install_real.bash'
    p.write_text(p.read_text().replace('ros_setup=/opt/ros/humble/setup.bash','ros_setup='+str(root/'absent_ros/setup.bash')))
    r=run(p,ws,env,'--dry-run');assert r.returncode!=0 and '--install-ros' in r.stdout
    r=run(p,ws,env,'--install-ros','--dry-run');assert r.returncode==0,r.stdout
    assert 'ros-infrastructure/ros-apt-source/releases/download/1.3.0/' in r.stdout
    assert 'ros-humble-ros-base' in r.stdout and 'ros-humble-desktop' not in r.stdout
    assert not ws.exists() and not log.exists()


def test_existing_non_git_dependency_is_preserved_and_rejected(sandbox):
    root,ws,env,log=sandbox
    target=ws/'src/Livox-SDK2';target.mkdir(parents=True);(target/'mine').write_text('keep')
    r=run(root/'tools/install_real.bash',ws,env)
    assert r.returncode!=0 and 'not a Git checkout' in r.stdout
    assert (target/'mine').read_text()=='keep'
    assert not any(e['tool'] in ('sudo','apt-get','cmake','colcon') for e in events(log))


def test_wrong_pinned_revision_is_rejected_before_changes(sandbox):
    root,ws,env,log=sandbox
    r=run(root/'tools/install_real.bash',ws,env);assert r.returncode==0,r.stdout
    target=ws/'src/fast_gicp'
    subprocess.run(['/usr/bin/git','-C',str(target),'config','rc2026.fakepin','0'*40],check=True)
    before=len(events(log));r=run(root/'tools/install_real.bash',ws,env)
    assert r.returncode!=0 and 'revision mismatch' in r.stdout
    assert not any(e['tool'] in ('sudo','apt-get','cmake','colcon') for e in events(log)[before:])


def test_unknown_installer_recipe_is_not_reapplied(sandbox):
    root,ws,env,log=sandbox
    r=run(root/'tools/install_real.bash',ws,env);assert r.returncode==0,r.stdout
    metadata=ws/'.rc2026_install/fast_gicp.json'
    data=json.loads(metadata.read_text());data['recipe']='0'*64;metadata.write_text(json.dumps(data))
    before=len(events(log));r=run(root/'tools/install_real.bash',ws,env)
    assert r.returncode!=0 and 'install recipe changed' in r.stdout
    assert not any(e['tool'] in ('sudo','apt-get','cmake','colcon') for e in events(log)[before:])


def test_network_clone_failure_does_not_publish_incomplete_checkout(sandbox):
    root,ws,env,log=sandbox
    git=Path(env['PATH'].split(':')[0])/'git'
    git.write_text(git.read_text().replace("source=json.loads(os.environ['FAKE_DEPENDENCIES'])[args[-2]]", "raise SystemExit('fixture: GitHub HTTPS clone failed')"))
    r=run(root/'tools/install_real.bash',ws,env)
    assert r.returncode!=0 and 'GitHub HTTPS clone failed' in r.stdout and 'install failed' in r.stdout
    assert not (ws/'src/Livox-SDK2').exists()
    assert not any(e['tool'] in ('cmake','colcon') for e in events(log))
