"""Check actual world geometry, reciprocal junction links and lane footprints."""
from pathlib import Path
import hashlib
import json
import math
import re
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from build_world import offset
from sync_world import read_nodes

ROOT = Path(__file__).resolve().parents[1]
EPS = .001


def transform(points, node):
    x, y, _ = node.get('translation', [0, 0, 0])
    angle = node.get('rotation', [0, 0, 1, 0])[3]
    c, s = math.cos(angle), math.sin(angle)
    return [(x+c*px-s*py, y+s*px+c*py) for px, py in points]


def road_strip(points, inner, outer):
    return Polygon(offset(points, inner) + list(reversed(offset(points, outer))))


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
        assert node.get('numberOfLanes') == road['number_of_lanes'] == 4, eid
        assert node.get('numberOfForwardLanes') == road['forward_lanes'] == 2, eid
        assert abs(node.get('width')-road['width']) < EPS, (eid, 'Width')
        if node.type == 'Road':
            assert node.get('splineSubdivision') == -1, (eid, 'Checker requires explicit unsmoothed waypoints')
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
        for field in ('startLine', 'endLine'):
            for url in node.get(field, []):
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
    surfaces += roundabout_surfaces(ring, data['junctions'])
    surface = unary_union(surfaces).buffer(.003)
    barrier_node = next(n for n in nodes if n.type == 'CrashBarrier')
    barrier = LineString(transform([p[:2] for p in barrier_node.coords('path')], barrier_node))
    assert barrier.is_ring and barrier_node.get('enableBoundingObject') == 'TRUE', 'Outer barrier must close and collide'
    xmin, xmax, ymin, ymax = data['bounds']
    assert all(xmin<x<xmax and ymin<y<ymax for x, y in barrier.coords), 'Barrier outside map bounds'
    assert barrier.distance(surface) > .17, 'Barrier profile intrudes into a road or junction'
    failures = []
    for lid, lane in data['lanes'].items():
        assert all(math.isfinite(v) for p in lane['points'] for v in p), (lid, 'Nonfinite lane coordinates')
        outside = LineString(lane['points']).buffer(.186, cap_style=1, join_style=1).difference(surface)
        if outside.area > .0001:
            failures.append({'lane': lid, 'outside_m2': round(outside.area, 6), 'bounds': list(outside.bounds)})
        assert lane['successors'], (lid, 'Dead end')
        for nxt in lane['successors']:
            assert nxt in data['lanes'] and math.dist(lane['points'][-1], data['lanes'][nxt]['points'][0])<EPS, (lid, nxt, 'Lane discontinuity')
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
    report = {'roads_checked': len(native_roads), 'junctions_checked': len(native_junctions), 'road_mouths_checked': mouths_checked,
              'incident_road_curbs_checked': curbs_checked, 'road_triangles_checked': triangles_checked, 'footprint_radius_m': .186, 'lanes_checked': len(data['lanes']), 'outside_road': failures,
              'directed_strong_connectivity': True, 'ordinary_junction_u_turns': 0,
              'world_sha256': hashlib.sha256(world_path.read_bytes()).hexdigest(), 'map_sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
              'note': 'Actual road/junction XY geometry and metadata QA; not vehicle dynamics or tunnel clearance verification.'}
    (ROOT/'validation').mkdir(exist_ok=True)
    (ROOT/'validation/geometry.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(check())
