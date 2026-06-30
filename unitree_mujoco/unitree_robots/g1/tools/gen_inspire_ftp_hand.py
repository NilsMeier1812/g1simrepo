#!/usr/bin/env python3
# Phase 1: FTP-Hand in MuJoCo, Finger AKTUIERT (Position-Servos) -> steuerbar.
# Body-Motoren (29) zuerst, dann 24 Finger-Position-Actuators in KANONISCHER
# Reihenfolge (muss mit bridge/inspire_hand/robot_state uebereinstimmen).
import os, math, re, xml.etree.ElementTree as ET
import pathlib; REPO=str(pathlib.Path(__file__).resolve().parents[4])
URDF=os.path.join(REPO,"g1pilot/description_files/urdf/g1_29dof_inspire_ftp.urdf")
BASE=os.path.join(REPO,"unitree_mujoco/unitree_robots/g1/g1_29dof.xml")
OUT =os.path.join(REPO,"unitree_mujoco/unitree_robots/g1/g1_29dof_inspire_ftp.xml")

# Kanonische Finger-Gelenk-Reihenfolge (identisch in robot_state.py / inspire_hand)
CANON=[f"{s}_{f}_{i}_joint" for s in ("left","right")
       for f,n in (("thumb",4),("index",2),("middle",2),("ring",2),("little",2))
       for i in range(1,n+1)]
FKP=2.0      # Position-Servo-Steifigkeit der Finger (tunebar)
FDAMP=0.05   # Gelenkdaempfung
FFRC=2.0     # Drehmoment-Limit der Finger-Servos

def rpy2quat(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2); cp,sp=math.cos(p/2),math.sin(p/2); cy,sy=math.cos(y/2),math.sin(y/2)
    return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)
def fmt(*v): return " ".join(f"{x:.8g}" for x in v)

u=ET.parse(URDF).getroot()
links={l.get('name'):l for l in u.findall('link')}
ch={}
for j in u.findall('joint'): ch.setdefault(j.find('parent').get('link'),[]).append(j)
RANGES={}; MESHES=set(); BODIES={}
SKIP=("force_sensor","hand_point_contact")

def mesh_base(link):
    l=links.get(link); 
    if l is None or l.find('visual') is None: return None
    m=l.find('visual/geometry/mesh'); return os.path.basename(m.get('filename')) if m is not None else None
def rpy_mat(r,p,y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp,   cp*sr,          cp*cr]]
def add_inertial(b,link):
    ine=links[link].find('inertial')
    if ine is None: return
    o=ine.find('origin'); I=ine.find('inertia')
    xyz=[float(t) for t in (o.get('xyz','0 0 0').split())] if o is not None else [0,0,0]
    rpy=[float(t) for t in (o.get('rpy','0 0 0').split())] if o is not None else [0,0,0]
    ixx,iyy,izz,ixy,ixz,iyz=[float(I.get(k)) for k in('ixx','iyy','izz','ixy','ixz','iyz')]
    M=[[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]]
    R=rpy_mat(*rpy)
    RM=[[sum(R[i][k]*M[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    Ib=[[sum(RM[i][k]*R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    # MuJoCo: fullinertia (im Body-Frame) -> KEIN quat zusaetzlich erlaubt. Inertia
    # daher per rpy in den Body-Frame rotiert (Ib = R*M*R^T), pos bleibt.
    el=ET.SubElement(b,'inertial'); el.set('pos',fmt(*xyz)); el.set('mass',ine.find('mass').get('value'))
    el.set('fullinertia',fmt(Ib[0][0],Ib[1][1],Ib[2][2],Ib[0][1],Ib[0][2],Ib[1][2]))
def add_geom(b,link):
    mb=mesh_base(link)
    if mb is None: return
    g=ET.SubElement(b,'geom'); g.set('type','mesh'); g.set('contype','0'); g.set('conaffinity','0')
    g.set('group','1'); g.set('density','0'); g.set('rgba','0.55 0.55 0.6 1'); g.set('mesh',os.path.splitext(mb)[0]); MESHES.add(mb)
def emit(parent, joint):
    child=joint.find('child').get('link')
    if any(s in joint.get('name') for s in SKIP): return
    o=joint.find('origin'); xyz=[float(t) for t in (o.get('xyz','0 0 0').split())]; rpy=[float(t) for t in (o.get('rpy','0 0 0').split())]
    b=ET.SubElement(parent,'body'); b.set('name',child); b.set('pos',fmt(*xyz))
    if any(abs(x)>1e-9 for x in rpy): b.set('quat',fmt(*rpy2quat(*rpy)))
    if joint.get('type')=='revolute':
        lim=joint.find('limit'); lo,hi=float(lim.get('lower')),float(lim.get('upper'))
        jt=ET.SubElement(b,'joint'); jt.set('name',joint.get('name')); jt.set('pos','0 0 0')
        jt.set('axis',joint.find('axis').get('xyz','0 0 1')); jt.set('range',fmt(lo,hi))
        jt.set('damping',str(FDAMP)); jt.set('frictionloss','0.002')
        RANGES[joint.get('name')]=(lo,hi)
    add_inertial(b,child); add_geom(b,child); BODIES[child]=b
    for cj in ch.get(child,[]): emit(b,cj)
def build(side):
    bj=[j for j in ch.get(f"{side}_wrist_yaw_link",[]) if j.get('name')==f"{side}_base_joint"][0]
    c=ET.Element('c'); emit(c,bj); return list(c)

t=ET.parse(BASE); root=t.getroot()
def body(n):
    for b in root.iter('body'):
        if b.get('name')==n: return b
for side in ("left","right"):
    wb=body(f"{side}_wrist_yaw_link")
    for g in list(wb.findall('geom')):
        if g.get('mesh')==f"{side}_rubber_hand": wb.remove(g)
    for el in build(side): wb.append(el)
# Mesh-Assets
asset=root.find('asset'); have={m.get('name') for m in asset.findall('mesh')}
for mb in sorted(MESHES):
    nm=os.path.splitext(mb)[0]
    if nm not in have: m=ET.SubElement(asset,'mesh'); m.set('name',nm); m.set('file',mb)
# Finger-Position-Actuators in kanonischer Reihenfolge anhaengen
act=root.find('actuator')
assert all(j in RANGES for j in CANON), [j for j in CANON if j not in RANGES]
for jn in CANON:
    lo,hi=RANGES[jn]
    p=ET.SubElement(act,'position'); p.set('name',jn); p.set('joint',jn)
    p.set('kp',str(FKP)); p.set('ctrlrange',fmt(lo,hi)); p.set('forcerange',fmt(-FFRC,FFRC))

# ── Phase 2: Fingerspitzen-Kollision + Touch-Sensoren ─────────────────────────
# Pro Fingerspitze (letztes Glied) ein kleiner Kollisions-Sphere + Touch-Site an der
# Pose des letzten URDF-Force-Sensor-Frames. contype/conaffinity=2 -> Finger
# kollidieren NUR mit Objekten, die Bit 2 setzen (greifbare Objekte), NICHT mit Boden
# (Bit 1) oder dem Roboter selbst -> keine Self/Floor-Kollision -> stabil.
FINGERS=(("thumb",4),("index",2),("middle",2),("ring",2),("little",2))
TOUCH=[]
sens=root.find('sensor')
for s_ in ("left","right"):
    for f,n in FINGERS:
        tip=f"{s_}_{f}_{n}"
        b=BODIES.get(tip)
        if b is None: continue
        fs=[j for j in ch.get(tip,[]) if 'force_sensor' in j.get('name')]
        if fs:
            fs.sort(key=lambda j:int(re.findall(r"(\d+)",j.get('name'))[-1]))
            o=fs[-1].find('origin'); pad=[float(t) for t in (o.get('xyz','0 0 0').split())]
        else:
            pad=[0.0,0.0,0.0]
        g=ET.SubElement(b,'geom'); g.set('type','sphere'); g.set('size','0.008')
        g.set('pos',fmt(*pad)); g.set('contype','2'); g.set('conaffinity','2')
        g.set('group','3'); g.set('rgba','0.9 0.2 0.2 1')
        st=ET.SubElement(b,'site'); st.set('name',f"{s_}_{f}_pad"); st.set('type','sphere')
        st.set('size','0.012'); st.set('pos',fmt(*pad)); st.set('group','4'); st.set('rgba','1 0 0 0.3')
        TOUCH.append((f"{s_}_{f}_touch", f"{s_}_{f}_pad"))
for sn,site in TOUCH:
    t_=ET.SubElement(sens,'touch'); t_.set('name',sn); t_.set('site',site)

root.set('model','g1_29dof_inspire_ftp')
ET.indent(t,space="  "); t.write(OUT,encoding='unicode',xml_declaration=False)
print("OK. finger actuators:", len(CANON), "| touch sensors:", len(TOUCH), "| total actuators:", len(act))
