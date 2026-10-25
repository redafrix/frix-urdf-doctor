"""Bounded isolated ROS/RViz rendering test. No robot/network/hardware actions."""
import os,sys,json,subprocess,time,signal,hashlib
from pathlib import Path
TRACK=Path(__file__).resolve().parent.parent;STUDIO=TRACK.parent.name=='tracks';ROOT=TRACK.parents[1] if STUDIO else TRACK;COLLISION='--collision' in sys.argv;OUT=ROOT/('data/t8/rviz-validation' if STUDIO else 'validation-output')
if COLLISION:OUT=OUT/'collision'
OUT.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,ROS_DOMAIN_ID='201',ROS_LOCALHOST_ONLY='1',DISPLAY=':193',LIBGL_ALWAYS_SOFTWARE='1',QT_X11_NO_MITSHM='1')
os.environ.update(env)
processes=[];files=[]
def spawn(name,args):
 log=open(OUT/(name+'.log'),'w');files.append(log);p=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT);processes.append(p);return p
try:
 display=spawn('xvfb',['Xvfb',':193','-screen','0','1280x900x24','-nolisten','tcp'])
 time.sleep(1)
 urdf=TRACK/'demo_description/urdf/cartpole.urdf';config=OUT/'robot-params.yaml';config.write_text('robot_state_publisher:\n  ros__parameters:\n    robot_description: '+json.dumps(urdf.read_text())+'\n')
 rsp=spawn('robot-state-publisher',['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher','--ros-args','--params-file',str(config)])
 jsp=spawn('joint-state-publisher',['/opt/ros/humble/lib/joint_state_publisher/joint_state_publisher'])
 rviz_config=OUT/'validated.rviz';rviz_text=(TRACK/'demo_description/rviz/cartpole.rviz').read_text()
 if COLLISION:rviz_text=rviz_text.replace('Visual Enabled: true','Visual Enabled: false').replace('Collision Enabled: false','Collision Enabled: true')
 rviz_config.write_text(rviz_text)
 rviz=spawn('rviz',['/opt/ros/humble/bin/rviz2','-d',str(rviz_config)])
 import rclpy
 from rclpy.node import Node
 from tf2_ros import Buffer,TransformListener
 rclpy.init();node=Node('frix_visual_validation');buffer=Buffer();listener=TransformListener(buffer,node);deadline=time.time()+35;transforms={}
 while time.time()<deadline:
  rclpy.spin_once(node,timeout_sec=.2)
  for child in ['cart','pole']:
   try:
    t=buffer.lookup_transform('slideBar',child,rclpy.time.Time());transforms[child]={'translation':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],'rotation':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}
   except Exception:pass
  if len(transforms)==2 and time.time()>deadline-24:break
 node.destroy_node();rclpy.shutdown()
 if len(transforms)!=2:raise RuntimeError('Both child frame transforms required')
 if any(p.poll() is not None for p in [rsp,jsp,rviz]):raise RuntimeError('ROS/RViz process exited')
 subprocess.run(['import','-window','root',str(OUT/'cartpole-rviz.png')],env=env,check=True,timeout=15)
 proof={'rendered':True,'transforms':transforms,'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'rviz_config_sha256':hashlib.sha256(rviz_config.read_bytes()).hexdigest(),'screenshot':str((OUT/'cartpole-rviz.png').relative_to(ROOT)),'collision_view':COLLISION,'renderer':'Xvfb, software OpenGL (LIBGL_ALWAYS_SOFTWARE=1)','ros':'Humble','isolation':'ROS domain 201; localhost only','human_visual_review':False,'limitation':'Software rendering and TF only. No hardware, physical correctness, inertia/unit certification or client delivery acceptance.'}
 (OUT/'proof.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof))
finally:
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
 for f in files:f.close()
