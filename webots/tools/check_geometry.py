"""Check actual world geometry, reciprocal junction links and lane footprints."""
from pathlib import Path
import hashlib
import json
import math
import re
from textwrap import dedent
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from build_world import offset
from sync_world import read_nodes, road_height

ROOT = Path(__file__).resolve().parents[1]
EPS = .001


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
        clearance = Point(position).distance(surface)-.02
        assert clearance > 0, (label, position, 'Column intrudes into a road/junction/roundabout', clearance)
        clearances.append(clearance)
    return {'sign_columns_checked': signs_checked, 'traffic_light_columns_checked': len(poses),
            'conservative_column_radius_m': .02, 'minimum_column_clearance_m': round(min(clearances), 6)}


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
    assert data['bounds'] == [-48, 48, -30, 48] and (xmax-xmin, ymax-ymin) == (96, 78), 'Expansion must enlarge the physical world to 96 x 78 m'
    prior = expansion['prior_bounds']
    assert prior == [-30, 24, -18, 18], 'Unexpected pre-expansion bounds'
    new_ids = expansion['new_road_ids']
    assert len(new_ids) == len(set(new_ids)) and len(new_ids) >= 20 and set(new_ids) <= set(native_roads), 'At least 20 distinct new physical roads are required'
    points = {eid: native_roads[eid].road_points() for eid in new_ids}
    outside = {eid: sum(not (prior[0] <= p[0] <= prior[1] and prior[2] <= p[1] <= prior[3]) for p in seq) for eid, seq in points.items()}
    assert sum(v > 0 for v in outside.values()) >= 10 and sum(outside.values()) >= 20, 'Road expansion stays inside the old world'
    middle = expansion['middle_road_ids']
    assert set(middle) == {'BLOCK_MID_TUNNEL', 'CENTRAL_1', 'CENTRAL_2', 'CENTRAL_3', 'ROUNDABOUT_EAST'}, 'Incomplete middle corridor'
    for eid in middle:
        road = native_roads[eid]
        assert road.get('numberOfLanes') == 8 and road.get('numberOfForwardLanes') == 4 and abs(road.get('width')-4.8)<EPS, (eid, 'Middle corridor requires four lanes per direction')
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
        assert span >= .5 and grade <= .16+1e-6, (eid, 'Northern hills require >= .5 m relief and <= 16% grade', span, grade)
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


def check():
    world_path, map_path = ROOT/'worlds/my_world.wbt', ROOT/'map.json'
    world = world_path.read_text(encoding='utf-8')
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
    assert set(native_junctions) == {n['world_id'] for n in data['junctions'].values()}, 'World/map junction IDs differ'
    roads = {e['id']: e for e in data['roads']}
    surfaces, actual_points, triangles_checked = [], {}, 0
    expected = {nid: set() for nid in native_junctions}
    for eid, road in roads.items():
        node = native_roads[eid]; points3 = node.road_points(); points = [p[:2] for p in points3]
        actual_points[eid] = points
        assert len(points) == len(road['points']), (eid, 'Point count')
        assert all(math.dist(a, b[:2]) < EPS for a, b in zip(points, road['points'])), (eid, 'World/map coordinates differ')
        assert len(points3) == len(road['elevations']) and all(abs(p[2]-z)<EPS for p, z in zip(points3, road['elevations'])), (eid, 'World/map elevations differ')
        lanes = node.get('numberOfLanes')
        assert lanes == road['number_of_lanes'] and lanes in (4, 8), eid
        assert node.get('numberOfForwardLanes') == road['forward_lanes'] == lanes/2, eid
        assert abs(node.get('width')-road['width']) < EPS and abs(road['width']-.6*lanes)<EPS, (eid, 'Width must keep .6 m lanes')
        if max(p[2] for p in points3) > .02:
            assert node.get('roadBoundingObject', 'FALSE') == 'TRUE', (eid, 'Raised road needs a physical road collider')
        if node.type == 'Road':
            assert node.get('splineSubdivision') == -1, (eid, 'Checker requires explicit unsmoothed waypoints')
        dividers = re.findall(r'\bRoadLine\s*\{([^{}]*)\}', node.get('lines', ''), re.S)
        assert len(dividers) == lanes-1, (eid, 'Wrong lane-divider count')
        doubles = [i for i, text in enumerate(dividers) if re.search(r'\btype\s+"double"', text)]
        assert doubles == [int(lanes/2)-1], (eid, 'Double center line must separate the two directions')
        for field, end in [('startJunction', 'from'), ('endJunction', 'to')]:
            nid = data['junctions'][road[end]]['world_id']
            assert node.get(field) == nid, (eid, field, nid)
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
            if kind in ('roundabout_port', 'connection'):
                assert not any(slots), (eid, field, 'Continuous seam/roundabout mouth must have no stop paint')
            else:
                incoming = list(range(int(lanes/2), int(lanes))) if field == 'startLine' else list(range(int(lanes/2)))
                assert len(slots) == lanes and [i for i, url in enumerate(slots) if 'stop_' in url] == incoming, (eid, field, 'Stop paint must cover every incoming lane')
            for url in slots:
                assert not url or (ROOT/'protos/vendor/projects/objects/road/protos'/url).resolve().is_file(), url
    for nid, node in native_junctions.items():
        assert set(node.get('connectedRoadIDs', [])) == expected[nid], (nid, 'Nonreciprocal road links')
    mouths_checked, curbs_checked = 0, 0
    for nid, junction in data['junctions'].items():
        node = native_junctions[junction['world_id']]
        assert {p['edge'] for p in junction['ports']} == {eid for eid, e in roads.items() if nid in (e['from'], e['to'])}, (nid, 'Stale ports')
        if junction['kind'] == 'roundabout_port':
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
    surface = unary_union(surfaces).buffer(.003)
    furniture_report = check_furniture(nodes, surface)
    barrier_node = next(n for n in nodes if n.type == 'CrashBarrier')
    barrier = LineString(transform([p[:2] for p in barrier_node.coords('path')], barrier_node))
    assert barrier.is_ring and barrier_node.get('enableBoundingObject') == 'TRUE', 'Outer barrier must close and collide'
    xmin, xmax, ymin, ymax = data['bounds']
    assert all(xmin<x<xmax and ymin<y<ymax for x, y in barrier.coords), 'Barrier outside map bounds'
    assert barrier.distance(surface) > .17, 'Barrier profile intrudes into a road or junction'
    failures = []
    for lid, lane in data['lanes'].items():
        assert all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in lane['points']), (lid, 'Lane route must retain finite XY coordinates')
        assert len(lane['elevations']) == len(lane['points']) and all(math.isfinite(z) for z in lane['elevations']), (lid, 'Lane height array must align with XY route')
        support = [roads[lane['road_id']]] if lane.get('road_id') else []
        assert all(abs(z-(road_height(p, support) if support else .008))<EPS for p, z in zip(lane['points'], lane['elevations'])), (lid, 'Lane elevation differs from its supporting road')
        outside = LineString(lane['points']).buffer(.186, cap_style=1, join_style=1).difference(surface)
        if outside.area > .0001:
            failures.append({'lane': lid, 'outside_m2': round(outside.area, 6), 'bounds': list(outside.bounds)})
        assert lane['successors'], (lid, 'Dead end')
        for nxt in lane['successors']:
            assert nxt in data['lanes'] and math.dist(lane['points'][-1], data['lanes'][nxt]['points'][0])<EPS, (lid, nxt, 'Lane discontinuity')
            assert abs(lane['elevations'][-1]-data['lanes'][nxt]['elevations'][0])<EPS, (lid, nxt, 'Lane elevation discontinuity')
            junction = data['junctions'].get(lane['to'])
            if lane.get('road_id') and junction and junction['kind'] != 'roundabout_port':
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
    report = {'roads_checked': len(native_roads), 'junctions_checked': len(native_junctions), 'road_mouths_checked': mouths_checked,
              'incident_road_curbs_checked': curbs_checked, 'road_triangles_checked': triangles_checked, 'footprint_radius_m': .186, 'lanes_checked': len(data['lanes']), 'outside_road': failures,
              'directed_strong_connectivity': True, 'ordinary_junction_u_turns': 0,
              'furniture': furniture_report,
              'expansion': expansion_report,
              'world_sha256': hashlib.sha256(world_path.read_bytes()).hexdigest(), 'map_sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
              'note': 'Actual road/junction XY geometry and metadata QA; not vehicle dynamics or tunnel clearance verification.'}
    (ROOT/'validation').mkdir(exist_ok=True)
    (ROOT/'validation/geometry.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(check())
