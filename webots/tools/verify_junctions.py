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


RUNTIME_HELPERS = r"""
def world_points(points, position, rotation):
    x,y,z,angle = rotation
    norm = math.sqrt(x*x+y*y+z*z)
    if angle == 0:
        return [[p[k]+position[k] for k in range(3)] for p in points]
    assert norm > 0, 'Invalid road rotation axis'
    axis = [x/norm,y/norm,z/norm]
    c,s = math.cos(angle),math.sin(angle)
    result = []
    for p in points:
        cross = [axis[1]*p[2]-axis[2]*p[1],axis[2]*p[0]-axis[0]*p[2],axis[0]*p[1]-axis[1]*p[0]]
        dot = sum(axis[k]*p[k] for k in range(3))
        result.append([position[k]+c*p[k]+s*cross[k]+(1-c)*dot*axis[k] for k in range(3)])
    return result

def peak_support(points):
    assert len(points) >= 3, 'Raised winding road needs interior waypoints'
    peak = max(p[2] for p in points[1:-1])
    assert peak > .05, 'Winding road has no raised interior floor'
    flat = [(a,b) for a,b in zip(points,points[1:]) if abs(a[2]-peak)<1e-6 and abs(b[2]-peak)<1e-6]
    if flat:
        a,b = max(flat,key=lambda pair:math.dist(pair[0],pair[1]))
        return [(a[k]+b[k])/2 for k in range(3)]
    return next(p for p in points[1:-1] if p[2] == peak)

def overview_position(bounds, field_of_view=.85, aspect=16/9):
    xmin,xmax,ymin,ymax = bounds
    assert xmax > xmin and ymax > ymin, 'Invalid overview bounds'
    height = max(xmax-xmin,(ymax-ymin)*aspect)/(2*math.tan(field_of_view/2))*1.15
    return [(xmin+xmax)/2,(ymin+ymax)/2,height]
"""


CONTROLLER = r"""
from controller import Supervisor
from pathlib import Path
import hashlib
import json
import math
import traceback

""" + RUNTIME_HELPERS + r"""

root = Path(__file__).resolve().parents[2]
manifest = json.loads((root/'junction-manifest.json').read_text(encoding='utf-8'))
map_data = json.loads((root/'map.json').read_text(encoding='utf-8'))
robot = Supervisor()
dt = int(robot.getBasicTimeStep())
children = robot.getRoot().getField('children')
report = {**manifest, 'status':'failed', 'errors':[], 'roads':[], 'junctions':[], 'medians':[], 'probes':[], 'images':[]}

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
    assert hashlib.sha256((root/'map.json').read_bytes()).hexdigest() == manifest['snapshot_map_sha256'], 'Snapshot map changed before load'
    step(2)
    roads, junctions, solids = {}, {}, {}
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        kind = node.getTypeName()
        if kind == 'Solid':
            name = field(node,'name').getSFString()
            assert name not in solids, 'Duplicate solid name '+name
            solids[name] = node
        if kind not in ('Road','StraightRoadSegment','Crossroad','RoadIntersection','Roundabout'):
            continue
        nid = field(node,'id').getSFString()
        assert nid, kind+' has an empty id'
        table = roads if kind in ('Road','StraightRoadSegment') else junctions
        assert nid not in table, 'Duplicate id '+nid
        table[nid] = node
    expected_roads = {road['id']:road for road in map_data['roads']}
    assert len(expected_roads) == len(map_data['roads']), 'Duplicate map road IDs'
    assert set(roads) == set(expected_roads), 'Loaded/map road IDs differ: '+str(set(roads)^set(expected_roads))
    expected_junctions = {node['world_id'] for node in map_data['junctions'].values() if not node.get('no_world_node')}
    assert set(junctions) == expected_junctions, 'Loaded/map junction IDs differ: '+str(set(junctions)^expected_junctions)
    for rid,node in roads.items():
        endpoints = [field(node,name).getSFString() for name in ('startJunction','endJunction')]
        assert all(endpoints), rid+' missing junction reference'
        expected = expected_roads[rid]
        assert endpoints == [map_data['junctions'][expected[key]]['world_id'] for key in ('from','to')], rid+' native junction endpoints differ from map'
        lanes = field(node,'numberOfLanes').getSFInt32()
        forward_lanes = field(node,'numberOfForwardLanes').getSFInt32()
        width = field(node,'width').getSFFloat()
        assert lanes == expected['number_of_lanes'], rid+' has wrong native lane count'
        assert forward_lanes == expected['forward_lanes'], rid+' has wrong native forward lane count'
        assert abs(width-expected['width']) < 1e-6, rid+' has wrong native road width'
        report['roads'].append({'id':rid,'type':node.getTypeName(),'startJunction':endpoints[0],'endJunction':endpoints[1],
                                'width':width,'numberOfLanes':lanes,'numberOfForwardLanes':forward_lanes})
        for key in ('from','to'):
            jnode = map_data['junctions'][expected[key]]
            if jnode.get('no_world_node'):
                continue
            jid = jnode['world_id']
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
    bridge_position = field(bridge,'translation').getSFVec3f()
    bridge_x,bridge_y,_ = bridge_position
    waypoints = field(tunnel,'wayPoints')
    points = [waypoints.getMFVec3f(i) for i in range(waypoints.getCount())]
    position = field(tunnel,'translation').getSFVec3f()
    points = world_points(points,position,field(tunnel,'rotation').getSFRotation())
    assert min(p[2] for p in points) < -.5, 'Tunnel is not below grade'
    floor = None
    for a,b in zip(points,points[1:]):
        if min(a[0],b[0]) <= bridge_x <= max(a[0],b[0]) and a[0] != b[0]:
            floor = a[2]+(b[2]-a[2])*(bridge_x-a[0])/(b[0]-a[0])
            break
    assert floor is not None and abs(floor+.55) < .015, 'Missing flat tunnel floor below bridge'
    assert abs(bridge_position[2]-.008) < .015, 'Unexpected bridge elevation'
    report['elevations'] = {'tunnel_floor_z':floor,'bridge_floor_z':bridge_position[2]}
    # ponytail: stationary boxes check support and side contacts, not vehicle
    # dynamics; add a driven robot only when validating traversal/steering.
    bridge_length = field(bridge,'length').getSFFloat()
    bridge_angle = field(bridge,'rotation').getSFRotation()[3]
    bridge_center = [bridge_x+bridge_length/2*math.cos(bridge_angle),bridge_y+bridge_length/2*math.sin(bridge_angle)]
    probes = [('tunnel-north',[bridge_x,.6,floor+.12],floor,.02),('tunnel-south',[bridge_x,-.6,floor+.12],floor,.02),
              ('bridge',[*bridge_center,bridge_position[2]+.12],bridge_position[2],.02)]
    for rid in ('NORTH_WINDING_W','NORTH_WINDING_E'):
        node = roads[rid]
        waypoints = field(node,'wayPoints')
        points = [waypoints.getMFVec3f(i) for i in range(waypoints.getCount())]
        points = world_points(points,field(node,'translation').getSFVec3f(),field(node,'rotation').getSFRotation())
        p = peak_support(points)
        assert field(node,'roadBoundingObject').getSFBool(), rid+' raised floor has no collider'
        probes.append((rid,[p[0],p[1],p[2]+.12],p[2],.04))
    if 'north_flyover' in map_data:
        flyover=map_data['north_flyover']
        deck=next(r for r in map_data['roads'] if r['id']==flyover['road'])
        cx,cy=flyover['center'];top=max(deck['elevations'])
        assert field(roads[flyover['road']],'roadBoundingObject').getSFBool(), 'Flyover has no deck collider'
        for i,offset in enumerate((-.9,-.3,.3,.9)):
            probes.append(('flyover-deck-'+str(i+1),[cx,cy+offset,top+.12],top,.02))
            probes.append(('flyover-underpass-'+str(i+1),[cx+offset,cy,.15],0,.02))
        for i,offset in enumerate((-2.1,-1.5,1.5,2.1)):
            probes.append(('flyover-ground-lane-'+str(i+1),[cx,cy+offset,.12],0,.02))
        for part in ('Deck','RampSupport-West','RampSupport-East'):
            name='Bridge-'+flyover['road']+'-'+part
            assert name in solids and field(solids[name],'boundingObject').getSFNode() is not None, name+' missing physical support'
        report['elevations']['flyover_floor_z']=top
    expected_medians = {m['road_id']:m for m in map_data.get('medians',[])}
    actual_medians = {name for name in solids if name.startswith('CentreMedian-')}
    assert actual_medians == {'CentreMedian-'+rid for rid in expected_medians}, 'Loaded/map median groups differ'
    for rid,median in expected_medians.items():
        node=solids['CentreMedian-'+rid]
        assert field(node,'boundingObject').getSFNode() is not None, rid+' median has no collider'
        lines=field(roads[rid],'lines')
        index=expected_roads[rid]['number_of_lanes']//2-1
        assert field(lines.getMFNode(index),'type').getSFString()=='none', rid+' median retains centre paint'
        flat=[(a,b) for section in median['sections'] for a,b in zip(section,section[1:]) if abs(a[2]-b[2])<1e-6]
        assert flat, rid+' median has no flat probe location'
        a,b=max(flat,key=lambda pair:math.dist(pair[0][:2],pair[1][:2]))
        p=[(a[k]+b[k])/2 for k in range(3)]
        support=p[2]+median['height']
        probes.append(('median-top-'+rid,[p[0],p[1],support+.12],support,.02))
        # The lane centre is .30 m from the road centre; its divider is .05 m wide on each side.
        length=math.dist(a[:2],b[:2]);normal=[-(b[1]-a[1])/length,(b[0]-a[0])/length]
        for side in (-1,1):
            probes.append(('median-inner-lane-'+rid+'-'+str(side),
                          [p[0]+side*.30*normal[0],p[1]+side*.30*normal[1],p[2]+.12],p[2],.02))
        report['medians'].append({'road_id':rid,'name':'CentreMedian-'+rid,'width_m':median['width'],
                                  'height_m':median['height'],'sections':len(median['sections'])})
        # Crossing gaps must be physically open, in addition to the static geometry check.
        road=expected_roads[rid]
        for endpoint in ('startLine','endLine'):
            paint=field(roads[rid],endpoint)
            if not any('cross' in paint.getMFString(i) for i in range(paint.getCount())):
                continue
            indices=(0,1) if endpoint=='startLine' else (-1,-2)
            a,b=[[*road['points'][i],road['elevations'][i]] for i in indices]
            distance=math.dist(a[:2],b[:2]);t=.2/distance
            p=[a[k]+t*(b[k]-a[k]) for k in range(3)]
            probes.append(('median-crossing-gap-'+rid+'-'+endpoint,[p[0],p[1],p[2]+.12],p[2],.02))
    if expected_medians:
        ring_lines=field(junctions['ROUNDABOUT'],'startRoadsLine')
        assert field(ring_lines.getMFNode(3),'type').getSFString()=='none', 'Roundabout sockets retain centre paint'
        ring_center=map_data['roundabout']['center']
        for nid,junction in map_data['junctions'].items():
            if junction['kind']!='roundabout_port':
                continue
            mouth=junction['position'];length=math.dist(mouth,ring_center)
            p=[mouth[k]+.15*(ring_center[k]-mouth[k])/length for k in (0,1)]
            support=.008+expected_medians[junction['ports'][0]['edge']]['height']
            probes.append(('median-roundabout-socket-'+nid,[*p,support+.12],support,.02))
    for i,(name,p,_,_) in enumerate(probes):
        height=.26 if name.startswith('flyover-underpass-') else .08
        children.importMFNodeFromString(-1, f'''DEF JUNCTION_PROBE_{i} Solid {{
          translation {p[0]} {p[1]} {p[2]} name "junction probe {i}"
          children [ Shape {{ geometry Box {{ size .16 .12 {height} }} }} ]
          boundingObject Box {{ size .16 .12 {height} }} physics Physics {{ density -1 mass .05 }} }}''')
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
    for i,(name,initial,support,tolerance) in enumerate(probes):
        node = robot.getFromDef('JUNCTION_PROBE_'+str(i))
        p = node.getPosition()
        contact_points = recorded[i]
        height=.26 if name.startswith('flyover-underpass-') else .08
        report['probes'].append({'name':name,'initial_position':initial,'position':p,
          'height_m':height,'expected_center_z':support+height/2,'contacts':contact_points})
        assert contact_points, name+' has no floor contact'
        assert abs(p[2]-(support+height/2)) < tolerance, name+' fell onto the wrong collider'
        assert math.dist(p[:2],initial[:2]) < (tolerance+.01), name+' shifted into a side collider'
        assert all(abs(c[2]-support) < tolerance+.005 for c in contact_points), name+' has wall/roof contact'
        node.remove()

    view = robot.getFromDef('OVERVIEW')
    assert view is not None, 'OVERVIEW Viewpoint missing'
    field(view,'orientation').setSFRotation([-.577350269,.577350269,.577350269,2.094395102])
    robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
    west_nodes = [node['position'] for name,node in map_data['junctions'].items() if name.startswith(('WEST_','BLOCK_'))]
    assert west_nodes, 'Western junctions missing from map'
    west_bounds = [min(p[0] for p in west_nodes)-4,max(p[0] for p in west_nodes)+4,
                   min(p[1] for p in west_nodes)-4,max(p[1] for p in west_nodes)+4]
    views=[('junction-overview.png',overview_position(map_data['bounds']),.85),
           ('junction-west-detail.png',overview_position(west_bounds,.8),.8)]
    if 'north_flyover' in map_data:
        views.append(('junction-flyover-detail.png',overview_position([-13,23,16,27],.8),.8))
    for name,p,fov in views:
        field(view,'position').setSFVec3f(p)
        field(view,'fieldOfView').setSFFloat(fov)
        step(16)
        robot.exportImage(str(root/name),100)
        step(2)
        assert (root/name).is_file() and (root/name).stat().st_size > 1000, 'Image export failed: '+name
        report['images'].append(name)
    if 'north_flyover' in map_data:
        # ENU Viewpoint looks along local +X with local +Z up.
        yaw=math.atan2(20,-25);pitch=math.atan2(10,math.hypot(25,20))
        qw=math.cos(yaw/2)*math.cos(pitch/2)
        q=[-math.sin(yaw/2)*math.sin(pitch/2),math.cos(yaw/2)*math.sin(pitch/2),math.sin(yaw/2)*math.cos(pitch/2)]
        norm=math.sqrt(sum(v*v for v in q))
        field(view,'orientation').setSFRotation([*(v/norm for v in q),2*math.acos(qw)])
        field(view,'position').setSFVec3f([32,1.5,10.4])
        field(view,'fieldOfView').setSFFloat(.9)
        step(16)
        name='junction-flyover-side.png'
        robot.exportImage(str(root/name),100)
        step(2)
        assert (root/name).is_file() and (root/name).stat().st_size>1000
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
        import math
        helpers = {'math':math}
        exec(RUNTIME_HELPERS,helpers)
        assert helpers['world_points']([[1,0,2]],[3,4,5],[0,0,1,0]) == [[4,4,7]]
        rotated = helpers['world_points']([[1,0,2]],[3,4,5],[0,0,1,math.pi/2])[0]
        assert math.dist(rotated,[3,5,7]) < 1e-9
        assert helpers['peak_support']([[0,0,0],[1,0,.6],[2,0,.6],[3,0,0]]) == [1.5,0,.6]
        assert helpers['peak_support']([[0,0,0],[1,0,.6],[2,0,0]]) == [1,0,.6]
        overview = helpers['overview_position']([-48,48,-30,48])
        assert overview[:2] == [0,9] and overview[2] > 170
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
                'snapshot_world_sha256':digest(world),'snapshot_map_sha256':digest(scratch/'map.json'),
                'scratch':str(scratch),'port':port}
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
        for key in ('nonce','source_world_sha256','snapshot_world_sha256','snapshot_map_sha256'):
            if received.get(key) != manifest[key]:
                raise RuntimeError('Unauthenticated runtime report: '+key)
        if digest(source) != manifest['source_world_sha256']:
            raise RuntimeError('Source world changed during verification')
        if digest(root/'map.json') != manifest['snapshot_map_sha256']:
            raise RuntimeError('Source map changed during verification')
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
