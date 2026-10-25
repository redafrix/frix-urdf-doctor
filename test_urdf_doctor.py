import importlib.util
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('frix_urdf_doctor',Path(__file__).with_name('urdf_doctor.py'));doctor=importlib.util.module_from_spec(spec);spec.loader.exec_module(doctor)
def run(tmp_path,text):
 p=tmp_path/'robot.urdf';p.write_text(text);before=p.read_bytes();r=doctor.lint(p);assert p.read_bytes()==before;return r
def test_bad_mass_axis_limits_and_graph_are_reported(tmp_path):
 text='''<robot name="bad"><link name="base"/><link name="arm"><inertial><mass value="0"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 0"/><limit lower="2" upper="1" effort="0" velocity="1"/></joint></robot>'''
 r=run(tmp_path,text);codes={i['code'] for i in r['issues']}
 assert {'nonpositive_mass','zero_axis','inverted_limits','nonpositive_limit'}<=codes and not r['passed']
def test_valid_fixed_tree_and_entity_rejection(tmp_path):
 r=run(tmp_path,'<robot name="example"><link name="base"/><link name="tip"/><joint name="j" type="fixed"><parent link="base"/><child link="tip"/></joint></robot>');assert r['passed']
 p=tmp_path/'evil.urdf';p.write_text('<!DOCTYPE robot [<!ENTITY secret SYSTEM "file:///etc/passwd">]><robot name="x"/>')
 with pytest.raises(ValueError,match='DTD'):doctor.lint(p)
def test_cycle_and_external_mesh_are_not_silently_accepted(tmp_path):
 text='<robot name="cycle"><link name="a"><visual><geometry><mesh filename="file:///etc/passwd"/></geometry></visual></link><link name="b"/><joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint><joint name="ba" type="fixed"><parent link="b"/><child link="a"/></joint></robot>'
 r=run(tmp_path,text);assert any(i['code']=='cycle' for i in r['issues']) and not r['passed']
def test_nonfinite_numbers_and_unexpanded_xacro_require_review(tmp_path):
 r=run(tmp_path,'<robot name="x" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="arm"/><link name="base"><inertial><mass value="nan"/></inertial></link></robot>')
 assert {'non_numeric','xacro_expansion_required'}<={i['code'] for i in r['issues']}

@pytest.mark.parametrize('name,links,joints',[('cartpole',3,2),('pendulum5',6,5),('r2d2',16,15)])
def test_three_pinned_public_descriptions(name,links,joints):
 import hashlib,json
 folder=Path(__file__).parent/'public-examples';p=folder/(name+'.urdf');record=next(x for x in json.loads((folder/'PROVENANCE.json').read_text()) if x['file']==p.name)
 assert hashlib.sha256(p.read_bytes()).hexdigest()==record['sha256']
 r=doctor.lint(p);assert r['links']==links and r['joints']==joints and r['passed']
