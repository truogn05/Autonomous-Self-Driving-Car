"""Check actual world geometry, reciprocal junction links and lane footprints."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import re
from textwrap import dedent
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import nearest_points, unary_union
from build_world import offset
from sync_world import NUMBER, read_nodes, road_height

ROOT = Path(__file__).resolve().parents[1]
EPS = .001


def closed_mesh(node):
    """Read the collision mesh and prove it is the visible, closed solid."""
    match = re.search(r'\bgeometry\s+DEF\s+(\w+)\s+IndexedFaceSet\s*\{', node.text)
    assert match and node.get('boundingObject') == 'USE '+match[1], (node.get('name'), 'Visual/collision mesh differs')
    assert node.get('translation', [0, 0, 0]) == [0, 0, 0] and abs(node.get('rotation', [0, 0, 1, 0])[3]) < EPS, 'Generated mesh must use world coordinates'
    source = re.sub(r'#[^\n]*', '', node.text[match.end():])
    coords = re.search(r'\bcoord\s+Coordinate\s*\{\s*point\s*\[([^]]*)\]', source)
    indices = re.search(r'\bcoordIndex\s*\[([^]]*)\]', source)
    assert coords and indices, (node.get('name'), 'Missing mesh vertices/faces')
    values = list(map(float, re.findall(NUMBER, coords[1])))
    assert len(values) % 3 == 0 and all(math.isfinite(v) for v in values)
    vertices = [values[i:i+3] for i in range(0, len(values), 3)]
    faces, face = [], []
    for value in map(int, re.findall(r'-?\d+', indices[1])):
        if value == -1:
            assert len(face) >= 3 and len(set(face)) == len(face)
            faces.append(face); face = []
        else:
            assert 0 <= value < len(vertices), 'Mesh index outside Coordinate array'
            face.append(value)
    assert not face and faces, 'Every mesh face must terminate with -1'
    edges = Counter((a, b) for f in faces for a, b in zip(f, f[1:]+f[:1]))
    assert all(count == 1 and edges[b, a] == 1 for (a, b), count in edges.items()), (node.get('name'), 'Mesh is open or inconsistently oriented')
    assert set(v for f in faces for v in f) == set(range(len(vertices))), 'Unused mesh vertices'
    return vertices, faces


def line_height(point, points):
    return road_height(point, [{'points': [p[:2] for p in points], 'elevations': [p[2] for p in points]}])


def transform(points, node):
    x, y, _ = node.get('translation', [0, 0, 0])
    angle = node.get('rotation', [0, 0, 1, 0])[3]
    c, s = math.cos(angle), math.sin(angle)
    return [(x+c*px-s*py, y+s*px+c*py) for px, py in points]


def road_strip(points, inner, outer):
    return Polygon(offset(points, inner) + list(reversed(offset(points, outer))))


def check_furniture(nodes, surface):
    columns = [(n.get('description'), n.get('translation')[:2]) for n in nodes
               if n.type == 'Solid' and (n.get('description', '').startswith(('SIGN_', 'YIELD_R_', 'DIRECTION_R_')) or n.get('description') == 'SPEED_LIMIT')]
    signs_checked = len(columns)
    system = next(n for n in nodes if re.match(r'DEF\s+SIGNAL_SYSTEM\s+Robot\b', n.text))
    poses = read_nodes(dedent(system.get('children')[1:-1]))
    assert len(poses) == 4 and all(n.type == 'Pose' for n in poses), 'SIGNAL_SYSTEM requires four outer lamp poses'
    for index, pose in enumerate(poses, 1):
        children = read_nodes(dedent(pose.get('children')[1:-1]))
        pole = next(n for n in children if n.get('name', '').startswith('TrafficLightPole-'))
        assert math.dist(pole.get('translation', [0, 0, 0])[:2], [0, 0]) < EPS, 'Lamp pole moved away from its outer pose'
        columns.append(('TrafficLightPole-'+str(index), transform([pose.get('translation')[:2]], system)[0]))
    clearances = []
    for label, position in columns:
        clearance = Point(position).distance(surface)-.12
        assert clearance > 0, (label, position, 'Column intrudes into a road/junction/roundabout', clearance)
        clearances.append(clearance)
    return {'sign_columns_checked': signs_checked, 'traffic_light_columns_checked': len(poses),
            'conservative_column_radius_m': .12, 'minimum_column_clearance_m': round(min(clearances), 6)}


def roundabout_surfaces(node, junctions):
    center = node.get('translation')[:2]
    outer, inner, width = node.get('outerRadius'), node.get('innerRadius'), node.get('startRoadsWidth')
    surfaces = [Point(center).buffer(outer, resolution=128).difference(Point(center).buffer(inner, resolution=128))]
    subdivision = int(node.get('subdivision', 16)*node.get('roadNumber', 4))
    theta = math.atan2(width/2, outer)
    theta2 = math.atan2(width/2+5*(width/2-outer*math.sin(theta)), outer)
    steps = round(theta2/(math.tau/subdivision))
    for junction in junctions.values():
        if junction['kind'] != 'roundabout_port':
            continue
        p = junction['position']; dist = math.dist(center, p)
        u = [(p[k]-center[k])/dist for k in (0, 1)]
        a = [center[k]+outer*u[k] for k in (0, 1)]
        surfaces.append(LineString([a, p]).buffer(width/2, cap_style=2))
        angle = math.atan2(u[1], u[0])
        for side in (-1, 1):
            mouth = (a[0]-side*width/2*u[1], a[1]+side*width/2*u[0])
            curve = [(center[0]+outer*math.cos(angle+side*j*math.tau/subdivision),
                      center[1]+outer*math.sin(angle+side*j*math.tau/subdivision)) for j in range(steps+1)]
            surfaces.append(Polygon([mouth]+curve))
    return surfaces


def check_expansion(data, nodes, native_roads):
    expansion = data['expansion']
    xmin, xmax, ymin, ymax = data['bounds']
    assert data['bounds'] == [-288, 288, -180, 288] and (xmax-xmin, ymax-ymin) == (576, 468), 'Expansion must enlarge the physical world to 576 x 468 m'
    prior = expansion['prior_bounds']
    assert prior == [-180, 144, -108, 108], 'Unexpected pre-expansion bounds'
    new_ids = expansion['new_road_ids']
    assert len(new_ids) == len(set(new_ids)) and len(new_ids) >= 20 and set(new_ids) <= set(native_roads), 'At least 20 distinct new physical roads are required'
    points = {eid: native_roads[eid].road_points() for eid in new_ids}
    outside = {eid: sum(not (prior[0] <= p[0] <= prior[1] and prior[2] <= p[1] <= prior[3]) for p in seq) for eid, seq in points.items()}
    assert sum(v > 0 for v in outside.values()) >= 10 and sum(outside.values()) >= 20, 'Road expansion stays inside the old world'
    middle = expansion['middle_road_ids']
    assert set(middle) == {'BLOCK_MID_TUNNEL', 'CENTRAL_1', 'CENTRAL_2', 'CENTRAL_3', 'ROUNDABOUT_EAST'}, 'Incomplete middle corridor'
    for eid in middle:
        road = native_roads[eid]
        assert road.get('numberOfLanes') == 8 and road.get('numberOfForwardLanes') == 4 and abs(road.get('width')-28.8)<EPS, (eid, 'Middle corridor requires four lanes per direction')
    for a, b in zip(middle, middle[1:]):
        assert native_roads[a].get('endJunction') == native_roads[b].get('startJunction'), (a, b, 'Disconnected middle corridor')
    north = {}
    assert set(expansion['north_road_ids']) == {'NORTH_WINDING_W', 'NORTH_WINDING_E'}, 'Both northern winding roads are required'
    for eid in expansion['north_road_ids']:
        seq = native_roads[eid].road_points()
        turns = []
        for a, b, c in zip(seq, seq[1:], seq[2:]):
            u, v = [b[k]-a[k] for k in (0, 1)], [c[k]-b[k] for k in (0, 1)]
            angle = math.atan2(u[0]*v[1]-u[1]*v[0], u[0]*v[0]+u[1]*v[1])
            if abs(angle) > .001:
                turns.append(1 if angle > 0 else -1)
        flips = sum(a != b for a, b in zip(turns, turns[1:]))
        span = max(p[2] for p in seq)-min(p[2] for p in seq)
        grade = max(abs(b[2]-a[2])/math.dist(a[:2], b[:2]) for a, b in zip(seq, seq[1:]))
        assert set(turns) == {-1, 1} and flips >= 1, (eid, 'Northern road must wind left and right')
        assert span >= 3.0 and grade <= .16+1e-6, (eid, 'Northern hills require >= 3.0 m relief and <= 16% grade', span, grade)
        north[eid] = {'xy_turn_sign_changes': flips, 'height_range_m': round(span, 6), 'maximum_grade': round(grade, 6)}
    ground = []
    for node in nodes:
        if node.type != 'Solid' or not node.get('name', '').upper().startswith('GROUND'):
            continue
        collider = node.get('boundingObject', '')
        match = re.search(r'\bBox\s*\{\s*size\s+([^}]+)\}', collider)
        assert match, (node.get('name'), 'Ground must have a Box collider')
        size = list(map(float, match[1].split()))
        x, y, z = node.get('translation', [0, 0, 0])
        assert len(size) == 3 and size[2] > 0 and abs(node.get('rotation', [0, 0, 1, 0])[3])<EPS, 'Ground boxes must have positive thickness and align to the world'
        assert abs(z+size[2]/2)<EPS, (node.get('name'), 'Ground surface elevation')
        ground.append(box(x-size[0]/2, y-size[1]/2, x+size[0]/2, y+size[1]/2))
    bounds = unary_union(ground).bounds
    assert len(bounds) == 4 and all(abs(a-b)<EPS for a, b in zip(bounds, [xmin, ymin, xmax, ymax])), ('Physical ground differs from map bounds', bounds)
    assert all(box(xmin, ymin, xmax, ymax).buffer(EPS).covers(part) for part in ground), 'Ground extends beyond declared world'
    return {'world_size_m': [xmax-xmin, ymax-ymin], 'new_roads_checked': len(new_ids),
            'new_roads_beyond_old_bounds': sum(v > 0 for v in outside.values()), 'new_waypoints_beyond_old_bounds': sum(outside.values()),
            'middle_lanes_per_direction': 4, 'northern_winding': north, 'physical_ground_bounds': list(bounds)}


def check_flyover(data, nodes, native_roads):
    if 'north_flyover' not in data:
        return None
    f=data['north_flyover']; roads={r['id']:r for r in data['roads']}
    assert not set(f['removed_roads']) & set(native_roads), 'The old parallel rows remain'
    assert not any(k.startswith('N1_') for k in data['junctions']), 'Old parallel junctions remain'
    for eid in f['approaches']:
        r=roads[eid]
        assert (r['number_of_lanes'],r['forward_lanes'],r['width'])==(8,4,28.8)
        assert all(abs(p[1]-129.0)<EPS for p in r['points']), 'Merged avenue centre'
    bridge=roads[f['road']]
    assert (bridge['number_of_lanes'],bridge['forward_lanes'],bridge['width'])==(4,2,14.4)
    assert all(abs(p[1]-129.0)<EPS for p in bridge['points']), 'Bridge must run straight through centre'
    assert bridge['from']==f['splits'][0] and bridge['to']==f['splits'][1]
    assert abs(bridge['elevations'][0]-.048)<EPS and abs(bridge['elevations'][-1]-.048)<EPS
    height=f['deck_height_m']; toes=f['toe_x']; flat=f['plateau_x']; thickness=f['deck_thickness_m']
    assert abs(height-6.192)<EPS and abs(thickness-.36)<EPS, 'Compact bridge dimensions differ from approved layout'
    assert all(abs(a-b)<EPS for a,b in zip(toes,[1.5,82.5])) and all(abs(a-b)<EPS for a,b in zip(flat,[19.5,64.5]))
    assert math.dist(bridge['points'][0],[toes[0],f['center'][1]])<EPS and math.dist(bridge['points'][-1],[toes[1],f['center'][1]])<EPS
    for eid in f['approaches']+f['ground_roads']:
        actual=native_roads[eid].road_points()
        for nid,point in [(roads[eid]['from'],actual[0]),(roads[eid]['to'],actual[-1])]:
            if nid in f['splits']:
                assert abs(point[0]-toes[f['splits'].index(nid)])<EPS and abs(point[2]-.048)<EPS, (eid,nid,'Actual road misses compact bridge split')
    assert .048+height-EPS <= max(bridge['elevations']) <= 6.240+EPS
    for p,z in zip(bridge['points'],bridge['elevations']):
        u = p[0] - f['center'][0]
        expected_z = .048 + (height / 2.0) * (1.0 + math.cos(math.pi * u / 40.5))
        assert abs(z - expected_z) < EPS, ('Arch profile differs', p, z, expected_z)
    grade=max(abs(b-a)/math.dist(p,q) for a,b,p,q in zip(bridge['elevations'],bridge['elevations'][1:],bridge['points'],bridge['points'][1:]))
    assert grade<.25, ('Flyover ramp grade',grade)
    for eid in f['ground_roads']:
        r=roads[eid];forward=2 if eid.endswith('_S') else 0
        assert (r['number_of_lanes'],r['forward_lanes'],r['width'])==(2,forward,7.2)
        assert all(abs(z-.048)<EPS for z in r['elevations'])
        assert all(abs(p[1]-(118.2 if forward else 139.8))<EPS for p in r['points'])
        assert native_roads[eid].get(('left' if forward else 'right')+'Border')=='FALSE', 'Ground curb blocks the rising inner lane'
    lanes=data['lanes']; split_links=0
    for lane in lanes.values():
        if lane.get('road_id') in f['approaches'] and lane['to'] in f['splits']:
            assert len(lane['successors'])==1, 'Each approach lane must retain one branch'
            target=lanes[lane['successors'][0]]
            expected=f['road'] if lane['lane_index']<=2 else next(eid for eid in f['ground_roads'] if eid.endswith(('_W_S' if lane['to']=='FLYOVER_W' else '_E_N')))
            assert target['road_id']==expected, ('Wrong bridge/ground split',lane,target)
            split_links+=1
    assert split_links==8
    ground_routes=unary_union([LineString(l['points']).buffer(1.116) for l in lanes.values()
        if l.get('junction')==f['ground_junction'] or l.get('road_id') in f['ground_roads']])
    structures={n.get('name'):n for n in nodes if n.get('name','').startswith('Bridge-NORTH_FLYOVER-')}
    assert set(structures)=={'Bridge-NORTH_FLYOVER-Deck','Bridge-NORTH_FLYOVER-RampSupport-West','Bridge-NORTH_FLYOVER-RampSupport-East'} and len(structures)==sum(n.get('name','').startswith('Bridge-NORTH_FLYOVER-') for n in nodes), 'Missing/duplicate bridge structures'
    assert not any(n.get('name','').startswith(('Flyover-Pier-','Flyover-Deck')) for n in nodes), 'Old bridge structures remain'
    deck=structures['Bridge-NORTH_FLYOVER-Deck']
    deck_vertices, deck_faces = closed_mesh(deck)
    assert len(deck_vertices) == 244 and len(deck_faces) == 242
    crossing_underside_min = min(v[2] for v in deck_vertices if abs(v[0] - f['center'][0]) <= 7.2)
    clearance = crossing_underside_min - .048
    assert clearance >= 5.0, ('Insufficient measured clearance below deck', clearance)
    inset=f['structural_inset_m']
    assert 0<inset<.048, 'Structural inset must avoid coplanar asphalt and keep toe supports above ground'
    support_count=0
    for side,limits in [('West',[toes[0],flat[0]]),('East',[flat[1],toes[1]])]:
        node=structures['Bridge-NORTH_FLYOVER-RampSupport-'+side]
        points=[[*p,z] for p,z in zip(bridge['points'],bridge['elevations']) if limits[0]-EPS<=p[0]<=limits[1]+EPS]
        expected=[[p[0],p[1]+offset_y,z] for p in points for offset_y,z in [(-7.2,0),(7.2,0),(-7.2,p[2]-inset),(7.2,p[2]-inset)]]
        vertices,faces=closed_mesh(node)
        assert len(vertices)==len(expected) and all(math.dist(a,b)<EPS for a,b in zip(vertices,expected)), (side,'Support does not fill the ramp from ground to road underside')
        assert len(faces)==4*(len(points)-1)+2, (side,'Incomplete ramp support faces')
        footprint=box(limits[0],f['center'][1]-7.2,limits[1],f['center'][1]+7.2)
        assert footprint.intersection(ground_routes).area<.000001, (side,'Ramp support blocks a ground movement footprint')
        support_count+=1
    polygon=Polygon(data['junctions'][f['ground_junction']]['polygon'])
    for lane in lanes.values():
        if lane.get('road_id')==f['road']:
            strip=road_strip(bridge['points'],-7.2,7.2).buffer(.018)
            assert LineString(lane['points']).buffer(1.116,cap_style=2).difference(strip).area<.0001, 'Flyover lane leaves deck'
        if lane.get('junction')==f['ground_junction']:
            assert max(lane['elevations'])<.06, 'Ground turn connects to the raised deck'
    crossing=polygon.intersection(road_strip(bridge['points'],-7.2,7.2))
    crossing_length=crossing.bounds[2]-crossing.bounds[0]
    length=sum(math.dist(a,b) for a,b in zip(bridge['points'],bridge['points'][1:]))
    assert length<2*crossing_length, ('Flyover must be shorter than twice the actual crossing',length,crossing_length)
    assert flat[0]<crossing.bounds[0] and flat[1]>crossing.bounds[2], 'Flat deck must extend beyond both crossing edges'
    assert all(road_height([x,f['center'][1]],[bridge]) >= 2.96 - EPS for x in [crossing.bounds[0],crossing.bounds[2]]), 'Ramp descends into crossing'
    return {'total_approach_lanes':8,'bridge_lanes_per_direction':2,'ground_lanes_per_direction':2,
            'split_lanes_checked':split_links,'deck_height_m':height,'maximum_ramp_grade':round(grade,6),
            'toe_distance_m':round(length,6),'crossing_length_m':round(crossing_length,6),'length_crossing_ratio':round(length/crossing_length,6),
            'deck_clearance_m':round(clearance,6),'ramp_supports_checked':support_count,'ground_routes_clear':True,
            'vehicle_height_verified':False}


def check_medians(data,nodes,native_roads,ring):
    roads={r['id']:r for r in data['roads']}
    expected={r['id'] for r in data['roads'] if r['number_of_lanes']==8}|{data['north_flyover']['road']}
    medians=data['medians']; by_id={m['road_id']:m for m in medians}
    assert len(medians)==len(by_id)==len(expected) and set(by_id)==expected, 'Median coverage must include every 8-lane road and the northern bridge'
    groups=[n for n in nodes if n.get('name','').startswith('CentreMedian-')]
    assert len(groups)==len(expected) and {n.get('name') for n in groups}=={'CentreMedian-'+eid for eid in expected}, 'Missing or duplicated median groups'
    groups={n.get('name')[13:]:n for n in groups}
    dividers=re.findall(r'\bRoadLine\s*\{([^{}]*)\}',ring.get('startRoadsLine',''),re.S)
    assert ring.get('startRoadsNumberOfLanes')==8 and len(dividers)==7 and re.search(r'\btype\s+"none"',dividers[3]), 'Roundabout centre paint remains'
    obstacles=[]; crossing_gaps=0; socket_extensions=0
    for eid,median in by_id.items():
        road=roads[eid]; seq=native_roads[eid].road_points(); centerline=LineString([p[:2] for p in seq])
        assert abs(median['width']-.60)<EPS and abs(median['height']-.60)<EPS, (eid,'Median width/height')
        assert len(median['sections'])==1 and len(median['sections'][0])>=2, (eid,'Unexpected median sections')
        section=median['sections'][0]; stations=[]
        for p in section:
            assert len(p)==3 and all(math.isfinite(v) for v in p), 'Invalid median baseline'
            station=centerline.project(Point(p[:2])); stations.append(station)
            # Socket extensions are checked separately below; road samples must retain their own XYZ.
            if EPS<station<centerline.length-EPS:
                assert Point(p[:2]).distance(centerline)<EPS and abs(p[2]-line_height(p[:2],seq))<EPS, (eid,'Median leaves its own road XYZ')
            else:
                assert abs(p[2]-seq[0 if station<EPS else -1][2])<EPS, (eid,'Median endpoint height')
        assert all(a<=b+EPS for a,b in zip(stations,stations[1:])), (eid,'Median baseline backtracks')
        for field,nid,p,anchor,nextp in [('startLine',road['from'],section[0],seq[0],seq[1]),('endLine',road['to'],section[-1],seq[-1],seq[-2])]:
            crossing=any('crossing' in url for url in native_roads[eid].get(field,[]))
            outward=[(nextp[k]-anchor[k])/math.dist(nextp[:2],anchor[:2]) for k in (0,1)]
            delta=[p[k]-anchor[k] for k in (0,1)]
            signed=sum(delta[k]*outward[k] for k in (0,1))
            assert abs(delta[0]*outward[1]-delta[1]*outward[0])<EPS, (eid,field,'Median endpoint off centre')
            if data['junctions'][nid]['kind']=='roundabout_port':
                assert not crossing and abs(signed+1.80)<EPS, (eid,'Median socket extension must be 1.80 m')
                socket_extensions+=1
            else:
                assert abs(signed-(2.70 if crossing else 0))<EPS, (eid,field,'Missing crossing gap or median stops short of road mouth')
                crossing_gaps+=int(crossing)
        left,right=offset([p[:2] for p in section],median['width']/2),offset([p[:2] for p in section],-median['width']/2)
        expected_vertices=[[x,y,z] for p,l,r in zip(section,left,right) for (x,y),z in [(r,p[2]),(l,p[2]),(l,p[2]+median['height']),(r,p[2]+median['height'])]]
        vertices,faces=closed_mesh(groups[eid])
        assert len(vertices)==len(expected_vertices) and all(math.dist(a,b)<EPS for a,b in zip(vertices,expected_vertices)), (eid,'Median collider mesh differs from its declared baseline/width/height')
        assert len(faces)==4*(len(section)-1)+2, (eid,'Incomplete median mesh')
        footprint=road_strip([p[:2] for p in section],-median['width']/2,median['width']/2)
        assert footprint.is_valid and footprint.area>0, (eid,'Invalid median footprint')
        pieces=[(Polygon([right[i],right[i+1],left[i+1],left[i]]),section[i:i+2]) for i in range(len(section)-1)]
        obstacles.append((eid,footprint,section,pieces))
    minimum=math.inf; roundabout_minimum=math.inf; lane_checks=0
    for lid,lane in data['lanes'].items():
        route=LineString(lane['points']); xyz=[[*p,z] for p,z in zip(lane['points'],lane['elevations'])]
        for eid,footprint,section,pieces in obstacles:
            distance=route.distance(footprint)
            if distance>3.0:
                continue
            on_route,on_median=nearest_points(route,footprint)
            lane_z=line_height(on_route.coords[0],xyz); base=line_height(on_median.coords[0],section)
            same_height=base-EPS<=lane_z<=base+.60+EPS
            if same_height:
                clearance=distance-1.116; minimum=min(minimum,clearance); lane_checks+=1
                if lane.get('junction','').startswith('R_') or lid.startswith('RING_'):
                    roundabout_minimum=min(roundabout_minimum,clearance)
                    assert clearance>=.12-EPS, (lid,eid,'Median intrudes into roundabout turn reserve',clearance)
            if distance>=1.116-EPS:
                continue
            # Linear mesh/route segments let us check the whole overlapping interval, not just one sample.
            for a,b in zip(xyz,xyz[1:]):
                segment=LineString([a[:2],b[:2]])
                if segment.distance(footprint)>=1.116-EPS:
                    continue
                for piece,baseline in pieces:
                    if segment.distance(piece)>=1.116-EPS:
                        continue
                    overlap=segment.intersection(piece.buffer(1.116))
                    intervals=[overlap] if overlap.geom_type in ('LineString','Point') else list(overlap.geoms)
                    for interval in intervals:
                        differences=[line_height(p,[a,b])-line_height(p,baseline) for p in interval.coords]
                        if differences:
                            assert min(differences)>.60+EPS or max(differences)<-EPS, (lid,eid,'Lane/connector footprint collides with median at the same elevation',list(interval.coords),differences)
    assert crossing_gaps==3 and socket_extensions==4, 'Unexpected crossing/socket coverage'
    assert minimum>=-EPS and math.isfinite(roundabout_minimum), 'Missing or colliding median route checks'
    return {'groups_checked':len(groups),'road_ids':sorted(by_id),'closed_collision_meshes':len(groups),
            'crossing_gaps_checked':crossing_gaps,'crossing_gap_m':2.70,'roundabout_socket_extensions_checked':socket_extensions,
            'socket_extension_m':1.80,'same_elevation_route_checks':lane_checks,'minimum_route_clearance_m':round(minimum,6),
            'minimum_roundabout_clearance_m':round(roundabout_minimum,6),'vertical_separation_checked':True}


def check():
    world_path, map_path = ROOT/'worlds/my_world.wbt', ROOT/'map.json'
    world = world_path.read_text(encoding='utf-8')
    if '# BEGIN NORTH_DISTRICT' in world:
        world = world.split('# BEGIN NORTH_DISTRICT')[0]
    data = json.loads(map_path.read_text(encoding='utf-8'))
    number = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
    for field, arity in [('translation', 3), ('rotation', 4)]:
        for match in re.finditer(r'\b'+field+r'\s+((?:'+number+r'[\s,]*)+)', world):
            assert len(re.findall(number, match[1])) == arity, (field, match[1])
    nodes = read_nodes(world)
    native_roads = {n.get('id'): n for n in nodes if n.type in ('Road', 'StraightRoadSegment')}
    native_junctions = {n.get('id'): n for n in nodes if n.type in ('Crossroad', 'RoadIntersection', 'Roundabout')}
    assert len(native_roads) == sum(n.type in ('Road', 'StraightRoadSegment') for n in nodes), 'Duplicate road IDs'
    assert len(native_junctions) == sum(n.type in ('Crossroad', 'RoadIntersection', 'Roundabout') for n in nodes), 'Duplicate junction IDs'
    assert set(native_roads) == {e['id'] for e in data['roads']}, 'World/map road IDs differ'
    assert set(native_junctions) == {n['world_id'] for n in data['junctions'].values() if not n.get('no_world_node')}, 'World/map junction IDs differ'
    roads = {e['id']: e for e in data['roads']}
    surfaces, actual_points, triangles_checked = [], {}, 0
    expected = {nid: set() for nid in native_junctions}
    for eid, road in roads.items():
        node = native_roads[eid]; points3 = node.road_points(); points = [p[:2] for p in points3]
        actual_points[eid] = points
        assert len(points) == len(road['points']), (eid, 'Point count')
        assert all(math.dist(a, b[:2]) < EPS for a, b in zip(points, road['points'])), (eid, 'World/map coordinates differ')
        assert len(points3) == len(road['elevations']) and all(abs(p[2]-z)<EPS for p, z in zip(points3, road['elevations'])), (eid, 'World/map elevations differ')
        lanes = node.get('numberOfLanes', 2 if node.type == 'StraightRoadSegment' else None)
        assert lanes == road['number_of_lanes'] and lanes in (2, 4, 8), eid
        forward=road['forward_lanes']
        assert node.get('numberOfForwardLanes') == forward and (forward in (0,2) if lanes==2 else forward==lanes/2), eid
        assert abs(node.get('width')-road['width']) < EPS and abs(road['width']-3.6*lanes)<EPS, (eid, 'Width must keep 3.6 m lanes')
        if max(p[2] for p in points3) > .12:
            assert node.get('roadBoundingObject', 'FALSE') == 'TRUE', (eid, 'Raised road needs a physical road collider')
        if node.type == 'Road':
            assert node.get('splineSubdivision') == -1, (eid, 'Checker requires explicit unsmoothed waypoints')
        dividers = re.findall(r'\bRoadLine\s*\{([^{}]*)\}', node.get('lines', ''), re.S)
        assert len(dividers) == lanes-1, (eid, 'Wrong lane-divider count')
        doubles = [i for i, text in enumerate(dividers) if re.search(r'\btype\s+"double"', text)]
        median_road = eid in {m['road_id'] for m in data.get('medians', [])}
        none = [i for i,text in enumerate(dividers) if re.search(r'\btype\s+"none"',text)]
        assert doubles == ([] if median_road or forward in (0,lanes) else [int(lanes/2)-1]), (eid, 'Direction separator paint does not match median coverage')
        assert none == ([int(lanes/2)-1] if median_road else []), (eid,'Only the median centre slot may be hidden')
        for field, end in [('startJunction', 'from'), ('endJunction', 'to')]:
            junc = data['junctions'][road[end]]
            nid = junc['world_id']
            assert node.get(field) == nid, (eid, field, nid)
            if not junc.get('no_world_node'):
                expected[nid].add(eid)
        half = road['width']/2
        left, right = offset(points, half), offset(points, -half)
        for i in range(len(points)-1):
            for a, b, c in [(right[i], right[i+1], left[i+1]), (right[i], left[i+1], left[i])]:
                area = ((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
                assert area > 1e-8, (eid, i, 'Inverted/degenerate road triangle', area)
                triangles_checked += 1
        strip = road_strip(points, -half, half)
        assert strip.is_valid and strip.area > 0, (eid, 'Invalid road strip')
        surfaces.append(strip)
        for field, end in [('startLine', 'from'), ('endLine', 'to')]:
            slots = node.get(field, [])
            kind = data['junctions'][road[end]]['kind']
            if kind in ('roundabout_port', 'connection', 'lane_split', 'dead_end'):
                assert not any(slots), (eid, field, 'Continuous seam/roundabout mouth must have no stop paint')
            else:
                incoming = list(range(int(forward), int(lanes))) if field == 'startLine' else list(range(int(forward)))
                assert len(slots) == lanes and [i for i, url in enumerate(slots) if 'stop_' in url] == incoming, (eid, field, 'Stop paint must cover every incoming lane')
            for url in slots:
                assert not url or (ROOT/'protos/vendor/projects/objects/road/protos'/url).resolve().is_file(), url
    for nid, node in native_junctions.items():
        assert set(node.get('connectedRoadIDs', [])) == expected[nid], (nid, 'Nonreciprocal road links')
    mouths_checked, curbs_checked = 0, 0
    for nid, junction in data['junctions'].items():
        node = native_junctions.get(junction['world_id'])
        assert {p['edge'] for p in junction['ports']} == {eid for eid, e in roads.items() if nid in (e['from'], e['to'])}, (nid, 'Stale ports')
        if junction['kind'] == 'roundabout_port':
            continue
        if junction['kind'] == 'lane_split':
            assert len(junction['ports']) in (3, 4) and (node is None or not node.coords('shape')), (nid,'Lane split has no crossing surface')
            if not junction.get('no_world_node'):
                assert all(abs(p['position'][0]-junction['position'][0])<EPS for p in junction['ports'])
            continue
        if junction['kind'] == 'connection':
            assert len(junction['ports']) == 2 and not node.coords('shape'), (nid, 'Logical bridge seam')
            assert all(math.dist(p['position'], junction['position'])<EPS for p in junction['ports']), (nid, 'Disconnected bridge seam')
            continue
        polygon = Polygon(junction['polygon'])
        assert polygon.is_valid and polygon.area > 0, (nid, 'Invalid junction polygon')
        if node.type == 'Crossroad':
            actual = Polygon(transform([(-p[1], -p[0]) for p in node.coords('shape')], node))
            assert actual.hausdorff_distance(polygon) < EPS, (nid, 'World/map junction shape differs')
            surfaces.append(actual)
        else:
            # Native RoadIntersection contains its central polygon and short approach roads.
            width = node.get('roadsWidth'); count = int(node.get('roadNumber', 4))
            radius = width/(2*math.sin(math.pi/count)); inner = width/(2*math.tan(math.pi/count))
            surfaces.append(Polygon(transform([(radius*math.sin(math.tau*i/count), radius*math.cos(math.tau*i/count)) for i in range(count)], node)))
            lengths = node.get('startRoadsLength', [5])
            for i in range(count):
                angle = math.tau*(i+.5)/count
                reach = lengths[0] if len(lengths) == 1 else lengths[i]
                endpoints = [(r*math.sin(angle), r*math.cos(angle)) for r in (inner, inner+reach)]
                surfaces.append(LineString(transform(endpoints, node)).buffer(width/2, cap_style=2))
        for port in junction['ports']:
            eid = port['edge']; road = roads[eid]; points = actual_points[eid]
            seq = points if road['from'] == nid else list(reversed(points))
            assert math.dist(seq[0], port['position']) < EPS, (nid, eid, 'Port moved from road end')
            ux, uy = [(seq[1][k]-seq[0][k])/math.dist(seq[0], seq[1]) for k in (0, 1)]
            assert math.dist([ux, uy], port['outward']) < EPS, (nid, eid, 'Incorrect port angle')
            x, y = seq[0]; half = road['width']/2
            cap = LineString([(x-uy*half, y+ux*half), (x+uy*half, y-ux*half)])
            assert cap.difference(polygon.buffer(EPS)).length < EPS, (nid, eid, 'Road mouth misses junction')
            mouths_checked += 1
            road_node = native_roads[eid]
            widths = road_node.get('roadBorderWidth', [road_node.get('startingRoadBorderWidth', .8), road_node.get('endingRoadBorderWidth', .8)])
            for side, label in [(-1, 'right'), (1, 'left')]:
                if road_node.get(label+'Border', 'TRUE') == 'FALSE' or road_node.get(label+'BorderBoundingObject', 'FALSE') != 'TRUE':
                    continue
                inner = offset(points, side*half)
                outer = [tuple(p[k]+(q[k]-p[k])*(1+widths[min(i, len(widths)-1)]/half) for k in (0, 1)) for i, (p, q) in enumerate(zip(points, inner))]
                curb = Polygon(inner+list(reversed(outer)))
                assert curb.is_valid and curb.intersection(polygon).area < .00001, (nid, eid, label, 'Road curb intrudes into junction')
                curbs_checked += 1
    ring = next(n for n in nodes if n.type == 'Roundabout')
    assert ring.get('numberOfLanes', 1) == data['roundabout']['ring_lanes'], 'World/map roundabout lane count differs'
    assert math.dist(ring.get('translation')[:2], data['roundabout']['center']) < EPS, 'World/map roundabout position differs'
    assert abs(ring.get('innerRadius')-data['roundabout']['inner_radius'])<EPS and abs(ring.get('outerRadius')-data['roundabout']['outer_radius'])<EPS, 'World/map roundabout radii differ'
    surfaces += roundabout_surfaces(ring, data['junctions'])
    surface = unary_union(surfaces).buffer(.018)
    furniture_report = check_furniture(nodes, surface)
    barrier_node = next(n for n in nodes if n.type == 'CrashBarrier')
    barrier = LineString(transform([p[:2] for p in barrier_node.coords('path')], barrier_node))
    assert barrier.is_ring and barrier_node.get('enableBoundingObject') == 'TRUE', 'Outer barrier must close and collide'
    xmin, xmax, ymin, ymax = data['bounds']
    assert all(xmin<x<xmax and ymin<y<ymax for x, y in barrier.coords), 'Barrier outside map bounds'
    assert barrier.distance(surface) > 1.02, 'Barrier profile intrudes into a road or junction'
    failures = []
    for lid, lane in data['lanes'].items():
        assert all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in lane['points']), (lid, 'Lane route must retain finite XY coordinates')
        assert len(lane['elevations']) == len(lane['points']) and all(math.isfinite(z) for z in lane['elevations']), (lid, 'Lane height array must align with XY route')
        support = [roads[lane['road_id']]] if lane.get('road_id') else []
        assert all(abs(z-(road_height(p, support) if support else .048))<EPS for p, z in zip(lane['points'], lane['elevations'])), (lid, 'Lane elevation differs from its supporting road')
        outside = LineString(lane['points']).buffer(1.116, cap_style=1, join_style=1).difference(surface)
        if outside.area > .0001:
            failures.append({'lane': lid, 'outside_m2': round(outside.area, 6), 'bounds': list(outside.bounds)})
        assert lane['successors'], (lid, 'Dead end')
        for nxt in lane['successors']:
            assert nxt in data['lanes'] and math.dist(lane['points'][-1], data['lanes'][nxt]['points'][0])<EPS, (lid, nxt, 'Lane discontinuity')
            assert abs(lane['elevations'][-1]-data['lanes'][nxt]['elevations'][0])<EPS, (lid, nxt, 'Lane elevation discontinuity')
            junction = data['junctions'].get(lane['to'])
            if lane.get('road_id') and junction and junction['kind'] not in ('roundabout_port', 'dead_end'):
                target = data['lanes'][nxt]
                exits = target['successors'] if target.get('junction') == lane['to'] else [nxt]
                assert all(data['lanes'][end].get('road_id') != lane['road_id'] for end in exits), (lid, nxt, 'Ordinary junction U-turn')
    for reverse in (False, True):
        graph = {lid: [] for lid in data['lanes']}
        for lid, lane in data['lanes'].items():
            for nxt in lane['successors']:
                graph[nxt if reverse else lid].append(lid if reverse else nxt)
        visited, todo = set(), [next(iter(graph))]
        while todo:
            lid = todo.pop()
            if lid not in visited:
                visited.add(lid); todo.extend(graph[lid])
        assert len(visited) == len(graph), ('Lane graph is not strongly connected', reverse, len(visited), len(graph))
    expansion_report = check_expansion(data, nodes, native_roads)
    flyover_report = check_flyover(data,nodes,native_roads)
    median_report = check_medians(data,nodes,native_roads,ring)
    report = {'roads_checked': len(native_roads), 'junctions_checked': len(native_junctions), 'road_mouths_checked': mouths_checked,
              'incident_road_curbs_checked': curbs_checked, 'road_triangles_checked': triangles_checked, 'footprint_radius_m': 1.116, 'lanes_checked': len(data['lanes']), 'outside_road': failures,
              'directed_strong_connectivity': True, 'ordinary_junction_u_turns': 0,
              'furniture': furniture_report,
              'north_flyover':flyover_report,
              'medians':median_report,
              'expansion': expansion_report,
              'world_sha256': hashlib.sha256(world_path.read_bytes()).hexdigest(), 'map_sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
              'note': 'Actual road/junction XYZ, median colliders, compact bridge structure and routing QA; vehicle height/dynamics are not verified.'}
    (ROOT/'validation').mkdir(exist_ok=True)
    (ROOT/'validation/geometry.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(check())
