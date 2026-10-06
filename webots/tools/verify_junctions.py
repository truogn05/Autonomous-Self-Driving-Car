"""Fresh Webots load, reciprocal junction metadata, tunnel collisions and images.

The check copies the current project to Temp; it never rewrites the user's world
or stops an existing Webots session. Run --self-check without launching Webots.
"""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import os
import re
import shutil
import socket
import subprocess
import tempfile
import uuid


CONTROLLER = r"""
from controller import Supervisor
from pathlib import Path
import hashlib
import json
import math
import traceback

root = Path(__file__).resolve().parents[2]
manifest = json.loads((root/'junction-manifest.json').read_text(encoding='utf-8'))
robot = Supervisor()
dt = int(robot.getBasicTimeStep())
children = robot.getRoot().getField('children')
report = {**manifest, 'status':'failed', 'errors':[], 'roads':[], 'junctions':[], 'probes':[], 'images':[]}

def field(node, name):
    value = node.getField(name)
    if value is None:
        raise ValueError(node.getTypeName()+' missing '+name)
    return value

def step(count):
    for _ in range(count):
        if robot.step(dt) == -1:
            raise RuntimeError('Simulation ended before verification completed')

try:
    actual = hashlib.sha256((root/'worlds/my_world.wbt').read_bytes()).hexdigest()
    assert actual == manifest['snapshot_world_sha256'], 'Snapshot changed before load'
    step(2)
    roads, junctions = {}, {}
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        kind = node.getTypeName()
        if kind not in ('Road','StraightRoadSegment','Crossroad','RoadIntersection','Roundabout'):
            continue
        nid = field(node,'id').getSFString()
        assert nid, kind+' has an empty id'
        table = roads if kind in ('Road','StraightRoadSegment') else junctions
        assert nid not in table, 'Duplicate id '+nid
        table[nid] = node
    assert len(roads) == 29, 'Expected 29 loaded road instances, got '+str(len(roads))
    for rid,node in roads.items():
        endpoints = [field(node,name).getSFString() for name in ('startJunction','endJunction')]
        assert all(endpoints), rid+' missing junction reference'
        report['roads'].append({'id':rid,'type':node.getTypeName(),'startJunction':endpoints[0],'endJunction':endpoints[1]})
        for jid in endpoints:
            assert jid in junctions, rid+' references missing '+jid
            links = field(junctions[jid],'connectedRoadIDs')
            ids = [links.getMFString(i) for i in range(links.getCount())]
            assert rid in ids, jid+' does not reciprocate '+rid
    for jid,node in junctions.items():
        links = field(node,'connectedRoadIDs')
        ids = [links.getMFString(i) for i in range(links.getCount())]
        assert ids and len(ids) == len(set(ids)), jid+' has empty/duplicate connected roads'
        report['junctions'].append({'id':jid,'type':node.getTypeName(),'connectedRoadIDs':ids})
        for rid in ids:
            assert rid in roads, jid+' references missing road '+rid
            assert jid in [field(roads[rid],name).getSFString() for name in ('startJunction','endJunction')], jid+' has unrelated road '+rid

    tunnel, bridge = roads['BLOCK_MID_TUNNEL'], roads['RECONNECTED_BRIDGE']
    waypoints = field(tunnel,'wayPoints')
    points = [waypoints.getMFVec3f(i) for i in range(waypoints.getCount())]
    position = field(tunnel,'translation').getSFVec3f()
    points = [[p[k]+position[k] for k in range(3)] for p in points]
    assert min(p[2] for p in points) < -.5, 'Tunnel is not below grade'
    floor = None
    for a,b in zip(points,points[1:]):
        if min(a[0],b[0]) <= -17 <= max(a[0],b[0]) and a[0] != b[0]:
            floor = a[2]+(b[2]-a[2])*(-17-a[0])/(b[0]-a[0])
            break
    assert floor is not None and abs(floor+.55) < .015, 'Missing flat tunnel floor at x=-17'
    bridge_position = field(bridge,'translation').getSFVec3f()
    assert abs(bridge_position[0]+17) < .001, 'Bridge is not above the moved tunnel'
    assert abs(bridge_position[2]-.008) < .015, 'Unexpected bridge elevation'
    report['elevations'] = {'tunnel_floor_z':floor,'bridge_floor_z':bridge_position[2]}
    # ponytail: stationary boxes check support and side contacts, not vehicle
    # dynamics; add a driven robot only when validating traversal/steering.
    probes = [('tunnel-north',[-17,.6,-.43],floor),('tunnel-south',[-17,-.6,-.43],floor),
              ('bridge',[-17,0,.128],bridge_position[2])]
    for i,(_,p,_) in enumerate(probes):
        children.importMFNodeFromString(-1, f'''DEF JUNCTION_PROBE_{i} Solid {{
          translation {p[0]} {p[1]} {p[2]} name "junction probe {i}"
          children [ Shape {{ geometry Box {{ size .16 .12 .08 }} }} ]
          boundingObject Box {{ size .16 .12 .08 }} physics Physics {{ density -1 mass .05 }} }}''')
        robot.getFromDef('JUNCTION_PROBE_'+str(i)).enableContactPointsTracking(dt)
    # Resting ODE bodies can sleep and stop emitting contacts. Retain contacts
    # during the fall, then check the final supported position separately.
    recorded = [[] for _ in probes]
    for _ in range(math.ceil(3*1000/dt)):
        step(1)
        for i in range(len(probes)):
            contacts = robot.getFromDef('JUNCTION_PROBE_'+str(i)).getContactPoints()
            if len(contacts) > len(recorded[i]):
                recorded[i] = [list(c.point) for c in contacts]
    for i,(name,initial,support) in enumerate(probes):
        node = robot.getFromDef('JUNCTION_PROBE_'+str(i))
        p = node.getPosition()
        contact_points = recorded[i]
        report['probes'].append({'name':name,'initial_position':initial,'position':p,
          'expected_center_z':support+.04,'contacts':contact_points})
        assert contact_points, name+' has no floor contact'
        assert abs(p[2]-(support+.04)) < .02, name+' fell onto the wrong collider'
        assert math.dist(p[:2],initial[:2]) < .03, name+' shifted into a side collider'
        assert all(abs(c[2]-support) < .025 for c in contact_points), name+' has wall/roof contact'
        node.remove()

    view = robot.getFromDef('OVERVIEW')
    assert view is not None, 'OVERVIEW Viewpoint missing'
    field(view,'orientation').setSFRotation([-.577350269,.577350269,.577350269,2.094395102])
    robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
    for name,p,fov in [('junction-overview.png',[-3,0,80],.85),('junction-west-detail.png',[-20,0,48],.8)]:
        field(view,'position').setSFVec3f(p)
        field(view,'fieldOfView').setSFFloat(fov)
        step(16)
        robot.exportImage(str(root/name),100)
        step(2)
        assert (root/name).is_file() and (root/name).stat().st_size > 1000, 'Image export failed: '+name
        report['images'].append(name)
    report['status'] = 'passed'
except Exception:
    report['errors'].append(traceback.format_exc())
finally:
    report['simulation_seconds'] = robot.getTime()
    (root/'junction-runtime.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('JUNCTION_CHECK '+json.dumps(report),flush=True)
    robot.simulationQuit(0 if report['status']=='passed' else 1)
"""


def check_world(source):
    pattern = r'(DEF\s+INSPECTOR\s+Robot\s*\{[^{}]*\bcontroller\s+)"world_inspector"'
    changed, count = re.subn(pattern, r'\1"junction_probe"', source)
    if count != 1:
        raise ValueError('Expected one INSPECTOR using world_inspector; found '+str(count))
    return changed


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--webots', default='E:/ProgramE/Webots/msys64/mingw64/bin/webots.exe')
    parser.add_argument('--timeout', type=int, default=180)
    parser.add_argument('--self-check', action='store_true')
    parser.add_argument('--prepare-only', action='store_true', help='Copy the snapshot without launching Webots')
    args = parser.parse_args()
    if args.self_check:
        ast.parse(CONTROLLER)
        sample = 'DEF INSPECTOR Robot { name "world inspector" controller "world_inspector" supervisor TRUE }'
        assert check_world(sample) == sample.replace('"world_inspector"','"junction_probe"')
        try:
            check_world(sample+sample)
        except ValueError:
            pass
        else:
            raise AssertionError('Duplicate inspector must be rejected')
        print('verify_junctions self-check PASS')
        return 0

    root = Path(__file__).resolve().parents[1]
    scratch = Path(tempfile.mkdtemp(prefix='junction-verification-'))
    source = root/'worlds/my_world.wbt'
    source_bytes = source.read_bytes()
    snapshot = check_world(source_bytes.decode('utf-8'))
    for directory in ('protos','controllers'):
        shutil.copytree(root/directory,scratch/directory,ignore=shutil.ignore_patterns('__pycache__'))
    (scratch/'worlds').mkdir()
    if (root/'worlds/textures').is_dir():
        shutil.copytree(root/'worlds/textures',scratch/'worlds/textures')
    for filename in ('map.json','map.svg'):
        if (root/filename).is_file():
            shutil.copy2(root/filename,scratch/filename)
    world = scratch/'worlds/my_world.wbt'
    world.write_text(snapshot,encoding='utf-8')
    controller = scratch/'controllers/junction_probe'
    controller.mkdir(exist_ok=True)
    (controller/'junction_probe.py').write_text(CONTROLLER,encoding='utf-8')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        port = sock.getsockname()[1]
    manifest = {'nonce':uuid.uuid4().hex,'source_world_sha256':hashlib.sha256(source_bytes).hexdigest(),
                'snapshot_world_sha256':digest(world),'scratch':str(scratch),'port':port}
    (scratch/'junction-manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(manifest,indent=2),flush=True)
    if args.prepare_only:
        return 0

    validation = root/'validation'
    validation.mkdir(exist_ok=True)
    result_file = validation/'junction-runtime.json'
    result = {**manifest,'status':'running','errors':[]}
    result_file.write_text(json.dumps(result,indent=2),encoding='utf-8')
    try:
        with (scratch/'webots.log').open('w',encoding='utf-8') as log:
            flags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0
            proc = subprocess.Popen([args.webots,'--batch','--minimize','--mode=fast','--stdout','--stderr',
                                     '--port='+str(port),str(world)],stdout=log,stderr=subprocess.STDOUT,
                                    cwd=scratch,creationflags=flags)
            try:
                exit_code = proc.wait(timeout=args.timeout)
            except subprocess.TimeoutExpired:
                if os.name=='nt':
                    subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],capture_output=True,check=False)
                else:
                    proc.kill()
                proc.wait(timeout=15)
                raise RuntimeError('Webots timeout; stopped only verification PID '+str(proc.pid))
        received = json.loads((scratch/'junction-runtime.json').read_text(encoding='utf-8'))
        for key in ('nonce','source_world_sha256','snapshot_world_sha256'):
            if received.get(key) != manifest[key]:
                raise RuntimeError('Unauthenticated runtime report: '+key)
        if digest(source) != manifest['source_world_sha256']:
            raise RuntimeError('Source world changed during verification')
        result = received
        log_text = (scratch/'webots.log').read_text(encoding='utf-8',errors='replace')
        if exit_code != 0:
            result['errors'].append('Webots exit code '+str(exit_code))
        if re.search(r'^\s*(?:ERROR|FATAL):',log_text,re.M|re.I):
            result['errors'].append('Webots log contains an ERROR/FATAL message')
        for name in result.get('images',[]):
            shutil.copy2(scratch/name,validation/name)
        if result['errors']:
            result['status'] = 'failed'
    except Exception as error:
        result['status'] = 'failed'
        result['errors'].append(str(error))
    finally:
        if (scratch/'webots.log').exists():
            shutil.copy2(scratch/'webots.log',validation/'junction-webots.log')
        result_file.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    return 0 if result['status']=='passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
