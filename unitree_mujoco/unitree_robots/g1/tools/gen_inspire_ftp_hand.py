#!/usr/bin/env python3
# Build-Tool: erzeugt g1_29dof_inspire_ftp.xml aus der FTP-URDF (Phase 0).
# Finger passiv (Feder haelt offen), visual-only, KEINE Actuators -> nu bleibt 29.
# Neu ausfuehren nach URDF-Aenderung:  python3 tools/gen_inspire_ftp_hand.py
import os, math, xml.etree.ElementTree as ET

import pathlib
REPO=str(pathlib.Path(__file__).resolve().parents[4])  # repo root
URDF=os.path.join(REPO,"g1pilot/description_files/urdf/g1_29dof_inspire_ftp.urdf")
BASE=os.path.join(REPO,"unitree_mujoco/unitree_robots/g1/g1_29dof.xml")
OUT =os.path.join(REPO,"unitree_mujoco/unitree_robots/g1/g1_29dof_inspire_ftp.xml")

def rpy2quat(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2)
    cp,sp=math.cos(p/2),math.sin(p/2)
    cy,sy=math.cos(y/2),math.sin(y/2)
    w=cr*cp*cy+sr*sp*sy
    x=sr*cp*cy-cr*sp*sy
    yq=cr*sp*cy+sr*cp*sy
    z=cr*cp*sy-sr*sp*cy
    return (w,x,yq,z)

def fmt(*v): return " ".join(f"{x:.8g}" for x in v)

u=ET.parse(URDF).getroot()
links={l.get('name'):l for l in u.findall('link')}
joints=u.findall('joint')
ch={}
for j in joints: ch.setdefault(j.find('parent').get('link'),[]).append(j)

def mesh_basename(link):
    l=links.get(link)
    if l is None: return None
    vis=l.find('visual')
    if vis is None: return None
    m=vis.find('geometry/mesh')
    if m is None: return None
    return os.path.basename(m.get('filename'))

def origin_of(j):
    o=j.find('origin')
    xyz=[float(t) for t in (o.get('xyz','0 0 0').split())] if o is not None else [0,0,0]
    rpy=[float(t) for t in (o.get('rpy','0 0 0').split())] if o is not None else [0,0,0]
    return xyz,rpy

def add_inertial(body, link):
    l=links[link]; ine=l.find('inertial')
    if ine is None: return
    o=ine.find('origin'); mass=ine.find('mass').get('value'); I=ine.find('inertia')
    xyz=[float(t) for t in (o.get('xyz','0 0 0').split())] if o is not None else [0,0,0]
    rpy=[float(t) for t in (o.get('rpy','0 0 0').split())] if o is not None else [0,0,0]
    q=rpy2quat(*rpy)
    el=ET.SubElement(body,'inertial')
    el.set('pos',fmt(*xyz)); el.set('quat',fmt(*q)); el.set('mass',mass)
    el.set('fullinertia',fmt(float(I.get('ixx')),float(I.get('iyy')),float(I.get('izz')),
                              float(I.get('ixy')),float(I.get('ixz')),float(I.get('iyz'))))

def add_visual_geom(body, link, rgba="0.6 0.6 0.6 1"):
    mb=mesh_basename(link)
    if mb is None: return
    name=os.path.splitext(mb)[0]
    g=ET.SubElement(body,'geom')
    g.set('type','mesh'); g.set('contype','0'); g.set('conaffinity','0')
    g.set('group','1'); g.set('density','0'); g.set('rgba',rgba); g.set('mesh',name)
    MESHES.add(mb)

MESHES=set()
SKIP_SUBSTR=("force_sensor","hand_point_contact")

def emit(parent_el, joint):
    """Emit a MuJoCo <body> for the child link of `joint` under parent_el. Recurse."""
    child=joint.find('child').get('link')
    if any(s in joint.get('name') for s in SKIP_SUBSTR): return  # Phase0: Force-Frames/EE ueberspringen
    xyz,rpy=origin_of(joint)
    q=rpy2quat(*rpy)
    b=ET.SubElement(parent_el,'body'); b.set('name',child); b.set('pos',fmt(*xyz))
    if any(abs(x)>1e-9 for x in rpy): b.set('quat',fmt(*q))
    if joint.get('type')=='revolute':
        lim=joint.find('limit'); ax=joint.find('axis').get('xyz','0 0 1')
        jt=ET.SubElement(b,'joint'); jt.set('name',joint.get('name'))
        jt.set('pos','0 0 0'); jt.set('axis',ax)
        jt.set('range',fmt(float(lim.get('lower')),float(lim.get('upper'))))
        # PASSIV: leichte Feder haelt offen (springref=lower), etwas Daempfung.
        jt.set('stiffness','0.3'); jt.set('damping','0.05')
        jt.set('springref',lim.get('lower'))
        jt.set('frictionloss','0.002')
    add_inertial(b, child)
    add_visual_geom(b, child)
    for cj in ch.get(child,[]):
        emit(b, cj)

def build_hand(side):
    # Wurzel = fixed *_base_joint -> *_base_link
    wrist=f"{side}_wrist_yaw_link"
    base_j=[j for j in ch.get(wrist,[]) if j.get('name')==f"{side}_base_joint"]
    assert base_j, f"kein {side}_base_joint"
    container=ET.Element('container')  # temporaer
    emit(container, base_j[0])
    return list(container)  # die base_link-Body-Elemente

# ---- Basis-MJCF laden, Rubber-Hand ersetzen ----
t=ET.parse(BASE); root=t.getroot()
def find_body(name):
    for b in root.iter('body'):
        if b.get('name')==name: return b
    return None

for side,rgba in (("left","0.55 0.55 0.6 1"),("right","0.55 0.55 0.6 1")):
    wb=find_body(f"{side}_wrist_yaw_link")
    assert wb is not None, f"{side}_wrist_yaw_link nicht gefunden"
    # alte rubber_hand-geoms entfernen
    for g in list(wb.findall('geom')):
        if g.get('mesh')==f"{side}_rubber_hand": wb.remove(g)
    # neue Hand anhaengen
    for el in build_hand(side):
        wb.append(el)

# ---- Mesh-Assets ergaenzen ----
asset=root.find('asset')
existing={m.get('name') for m in asset.findall('mesh')}
for mb in sorted(MESHES):
    name=os.path.splitext(mb)[0]
    if name in existing: continue
    m=ET.SubElement(asset,'mesh'); m.set('name',name); m.set('file',mb)

# Modellname anpassen
root.set('model','g1_29dof_inspire_ftp')
ET.indent(t, space="  ")
t.write(OUT, encoding='unicode', xml_declaration=False)

# Mesh-Liste fuer Copy ausgeben
print("NEEDED_MESHES:", " ".join(sorted(MESHES)))
print("OUT:", OUT)
print("hand revolute joints emitted:", sum(1 for _ in root.iter('joint') if any(k in (_.get('name') or '') for k in ('thumb','index','middle','ring','little'))))
