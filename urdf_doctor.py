"""Read-only URDF checks. A lint report is not physical/hardware certification."""
import argparse,json,math,re,xml.etree.ElementTree as ET
from pathlib import Path
def lint(path):
 path=Path(path);raw=path.read_bytes()
 if len(raw)>2_000_000:raise ValueError('URDF exceeds 2 MB input limit')
 if re.search(br'<!DOCTYPE|<!ENTITY',raw,re.I):raise ValueError('DTD/entities are not accepted')
 root=ET.fromstring(raw)
 if root.tag!='robot':raise ValueError('Expected a robot root')
 issues=[]
 def add(level,code,item,detail):issues.append({'level':level,'code':code,'item':item,'detail':detail})
 def number(text,item):
  try:
   value=float(text)
   if not math.isfinite(value):raise ValueError()
   return value
  except (TypeError,ValueError):add('error','non_numeric',item,'Expected a finite number; expand xacro expressions explicitly first.');return None
 def vector(text,item,default):
  parts=(text or default).split()
  if len(parts)!=3:add('error','vector_size',item,'Expected three components.');return None
  result=[number(v,item) for v in parts];return result if None not in result else None
 links={};parents={};edges={}
 dynamic_children={j.find('child').get('link') for j in root.findall('joint') if j.get('type')!='fixed' and j.find('child') is not None}
 for link in root.findall('link'):
  name=link.get('name','')
  if not name or name in links:add('error','link_name',name,'Missing or duplicate link name.')
  links[name]=link;edges[name]=[]
  inertial=link.find('inertial')
  if inertial is None:add('warning','missing_inertia',name,'No inertial block; review intended fixed/base/dynamic use.')
  else:
   mass=inertial.find('mass');value=number(mass.get('value') if mass is not None else None,name)
   if value is not None and value<=0:
    if value==0 and name not in dynamic_children:add('warning','static_zero_mass',name,'Zero-mass fixed/base link: verify intended static use; no physical certification.')
    else:add('error','nonpositive_mass',name,'Dynamic mass must be positive.')
   tensor=inertial.find('inertia')
   if tensor is None:add('error','missing_tensor',name,'Missing inertia tensor.')
   else:
    values=[number(tensor.get(k),name) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
    if None not in values:
     a,b,c,d,e,f=values;det=a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d
     if a<=0 or a*b-d*d<=0 or det<=0:add('error','invalid_inertia_tensor',name,'Tensor must be symmetric positive definite.')
     elif a+b<c-1e-12 or a+c<b-1e-12 or b+c<a-1e-12:add('warning','inertia_triangle',name,'Review principal moments and units; this is only a conservative tensor check.')
  if link.find('visual') is not None and link.find('collision') is None:add('warning','missing_collision',name,'Visual geometry has no collision geometry.')
  for origin in link.findall('.//origin'):
   vector(origin.get('xyz'),name,'0 0 0');vector(origin.get('rpy'),name,'0 0 0')
  for mesh in link.findall('.//mesh'):
   filename=mesh.get('filename','')
   if not filename:add('error','missing_mesh_path',name,'Mesh filename is empty.')
   elif filename.startswith(('http:','https:')) or Path(filename).is_absolute() or '..' in Path(filename).parts:add('warning','nonportable_mesh_path',name,'Remote, absolute or parent traversal path; no file/network access attempted.')
   else:add('warning','mesh_resolution_required',name,'Resolve mesh in the agreed ROS package environment; no external path is read by this linter.')
 joints=set()
 for joint in root.findall('joint'):
  name=joint.get('name','');kind=joint.get('type')
  if not name or name in joints:add('error','joint_name',name,'Missing or duplicate joint name.')
  joints.add(name);parent=joint.find('parent');child=joint.find('child');p=parent.get('link') if parent is not None else None;c=child.get('link') if child is not None else None
  if p not in links or c not in links:add('error','unknown_link',name,'Joint parent or child does not name a declared link.')
  else:
   if c in parents:add('error','multiple_parents',c,'Link has more than one parent joint.')
   parents[c]=p;edges[p].append(c)
  if kind not in {'fixed','revolute','continuous','prismatic','floating','planar'}:add('error','joint_type',name,'Unknown joint type.')
  if kind in {'revolute','continuous','prismatic','planar'}:
   axis=joint.find('axis');v=vector(axis.get('xyz') if axis is not None else None,name,'1 0 0')
   if v:
    norm=math.sqrt(sum(x*x for x in v))
    if norm<1e-9:add('error','zero_axis',name,'Axis cannot be zero.')
    elif abs(norm-1)>.001:add('warning','nonunit_axis',name,'Review axis normalization and frame convention.')
  if kind in {'revolute','prismatic','continuous'}:
   limit=joint.find('limit')
   if limit is None:
    if kind=='continuous':add('warning','missing_control_limits',name,'Continuous URDF joint permits omitted limits; controller effort/velocity still need explicit review.')
    else:add('error','missing_limits',name,'Revolute/prismatic joint requires a limit element.')
   else:
    for key in ('effort','velocity'):
     value=number(limit.get(key),name)
     if value is not None and value<=0:add('error','nonpositive_limit',name,key+' must be positive.')
    if kind!='continuous':
     low=number(limit.get('lower'),name);high=number(limit.get('upper'),name)
     if low is not None and high is not None and low>=high:add('error','inverted_limits',name,'Lower limit must be smaller than upper limit.')
  origin=joint.find('origin')
  if origin is not None:vector(origin.get('xyz'),name,'0 0 0');vector(origin.get('rpy'),name,'0 0 0')
 roots=set(links)-set(parents)
 if len(roots)!=1:add('error','root_count',root.get('name',''),'Expected exactly one root link; found '+str(len(roots))+'.')
 visited=set();active=set()
 def visit(name):
  if name in active:add('error','cycle',name,'Joint graph contains a cycle.');return
  if name in visited:return
  active.add(name)
  for child in edges[name]:visit(child)
  active.remove(name);visited.add(name)
 for name in links:visit(name)
 if any('xacro' in element.tag for element in root.iter()):add('error','xacro_expansion_required',root.get('name',''),'Expand reviewed xacro in an isolated ROS environment before final validation.')
 return {'robot':root.get('name'),'links':len(links),'joints':len(joints),'passed':not any(x['level']=='error' for x in issues),'issues':issues,'limitation':'Static lint only. Reda must review physical correctness, units and frames before delivery. No hardware or safety certification.'}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('file');p.add_argument('--output');a=p.parse_args();result=lint(a.file);text=json.dumps(result,indent=2,ensure_ascii=False)+'\n'
 if a.output:Path(a.output).write_text(text)
 else:print(text,end='')
 raise SystemExit(0 if result['passed'] else 1)
