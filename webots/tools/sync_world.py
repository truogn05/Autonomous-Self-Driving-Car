"""Synchronize junctions and routing from the saved WBT, preserving GUI edits."""
import json
import math
import re
from pathlib import Path
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union
import build_world as g

ROOT = Path(__file__).resolve().parents[1]
TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|#[^\n]*|[{}\[\]]|[^{}\[\]\s,]+')
NUMBER = r'[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?'


class Node:
    def __init__(self, type_, text, start=0, end=0):
        self.type, self.text, self.start, self.end = type_, text, start, end
        self.fields = {m[1]: m for m in re.finditer(
            r'^  (\w+)\s+(.+?)(?=^  \w+\s|^})', text, re.M | re.S)}

    def get(self, key, default=None):
        if key not in self.fields:
            return default
        value = self.fields[key][2].strip()
        if value.startswith('"'):
            return json.loads(value)
        if re.fullmatch(NUMBER, value):
            return float(value)
        if value.startswith('[') and not re.search(r'[{}]', value):
            if '"' in value:
                return re.findall(r'"([^"]*)"', value)
            return [float(v) for v in re.findall(NUMBER, value)]
        if re.fullmatch(rf'{NUMBER}(?:\s+{NUMBER})+', value):
            return list(map(float, value.split()))
        return value

    def coords(self, key):
        values = self.get(key, [])
        assert len(values) % 3 == 0, (self.get('id'), key)
        return [values[i:i+3] for i in range(0, len(values), 3)]

    def set(self, key, value):
        value = f'  {key} {value}\n'
        if key in self.fields:
            m = self.fields[key]
            self.text = self.text[:m.start()] + value + self.text[m.end():]
        else:
            at = self.text.index('{') + 1
            self.text = self.text[:at] + '\n' + value.rstrip('\n') + self.text[at:]
        self.__init__(self.type, self.text, self.start, self.end)

    def road_points(self):
        p = self.get('translation', [0, 0, 0])
        rot = self.get('rotation', [0, 0, 1, 0])
        assert abs(rot[3]) < 1e-8 or (abs(rot[0])+abs(rot[1]) < 1e-8 and abs(rot[2]-1) < 1e-8)
        angle = rot[3]
        pts = self.coords('wayPoints') if self.type == 'Road' else [[0, 0, 0], [self.get('length'), 0, 0]]
        return [[p[0]+x*math.cos(angle)-y*math.sin(angle),
                 p[1]+x*math.sin(angle)+y*math.cos(angle), p[2]+z] for x, y, z in pts]


def read_nodes(text):
    tokens = list(TOKEN.finditer(text))
    nodes, depth, begin = [], 0, None
    for i, tok in enumerate(tokens):
        if tok[0] == '{':
            if depth == 0:
                begin = i
            depth += 1
        elif tok[0] == '}':
            depth -= 1
            assert depth >= 0, 'Unbalanced WBT braces'
            if depth == 0:
                name = tokens[begin-1]
                start = name.start()
                if begin >= 3 and tokens[begin-3][0] == 'DEF':
                    start = tokens[begin-3].start()
                nodes.append(Node(name[0], text[start:tok.end()], start, tok.end()))
    assert depth == 0, 'Unbalanced WBT braces'
    return nodes


def rewrite(text, nodes):
    for n in reversed(nodes):
        text = text[:n.start] + n.text + text[n.end:]
    return text


def mouth(p, u, width):
    side = g.mul(g.normal(u), width/2)
    return [g.add(p, side), g.add(p, g.mul(side, -1))]


def native_id(nid):
    return 'ROUNDABOUT' if nid.startswith('R_') else ('SIGNAL_CROSS' if nid == 'SIGNAL_CROSS' else 'J_'+nid)


def curb_node(nid, n):
    boundary = Polygon(n['polygon']).boundary
    caps = unary_union([LineString(mouth(p['position'], p['outward'], p['width'])).buffer(.00002)
                        for p in n['ports']])
    parts = boundary.difference(caps)
    parts = [parts] if parts.geom_type == 'LineString' else list(parts.geoms)
    poses = []
    # Polygon vertices are CCW: the curb sits outside, with an open road mouth.
    for part in parts:
        for a, b in zip(part.coords, list(part.coords)[1:]):
            length = math.dist(a, b)
            if length < .001:
                continue
            u = g.direction(a, b)
            center = g.add(g.mul(g.add(a, b), .5), g.mul(g.normal(u), -.07))
            poses.append(f'Pose {{ translation {g.vec((*center, .0255))} rotation 0 0 1 {g.fmt(math.atan2(u[1],u[0]))} children [ Box {{ size {g.vec((length, .14, .035))} }} ] }}')
    shapes = [pose.replace('Box {', 'Shape { appearance '+g.PAVEMENT+' geometry Box {').replace(' } ] }', ' } } ] }') for pose in poses]
    return '\nSolid {\n  name "JunctionCurbs-'+nid+'"\n  children [ '+ ' '.join(shapes)+' ]\n  boundingObject Group { children [ '+' '.join(poses)+' ] }\n}\n'


def main():
    path = ROOT/'worlds/my_world.wbt'
    source = path.read_text(encoding='utf-8')
    nodes = read_nodes(source)
    data = json.loads((ROOT/'map.json').read_text(encoding='utf-8'))
    junctions = data['junctions']
    junctions.pop('BLOCK_CROSS', None)
    bridge = next(n for n in nodes if n.get('id') == 'RECONNECTED_BRIDGE')
    for key, p in zip(('BRIDGE_N', 'BRIDGE_S'), (bridge.road_points()[0], bridge.road_points()[-1])):
        junctions[key] = {'position': p[:2], 'kind': 'connection', 'trim': 0}
    for n in junctions.values():
        n['ports'] = []
        n.pop('polygon', None)
    old_roads = {r['id']: r for r in data['roads']}
    routing = {'BLOCK_MID_TUNNEL': ('WEST_MIDDLE', 'SIGNAL_CROSS'),
               'RECONNECTED_BRIDGE': ('BRIDGE_N', 'BRIDGE_S'),
               'RECONNECTED_NORTH': ('BLOCK_NORTH', 'BRIDGE_N'),
               'RECONNECTED_SOUTH': ('BRIDGE_S', 'BLOCK_SOUTH')}
    roads = []
    for node in nodes:
        if node.type not in ('Road', 'StraightRoadSegment'):
            continue
        eid = node.get('id')
        a, b = routing.get(eid, (old_roads[eid]['from'], old_roads[eid]['to']) if eid in old_roads else (None, None))
        assert a and b, eid
        xyz = node.road_points()
        pts = [p[:2] for p in xyz]
        for nid, p, q in [(a, pts[0], pts[1]), (b, pts[-1], pts[-2])]:
            junctions[nid]['ports'].append({'edge': eid, 'position': p, 'outward': g.direction(p, q), 'width': node.get('width')})
        node.set('startJunction', json.dumps(native_id(a)))
        node.set('endJunction', json.dumps(native_id(b)))
        # No stop/crossing paint at a continuous bridge seam.
        if a.startswith('BRIDGE_'):
            node.set('startLine', '[]')
        if b.startswith('BRIDGE_'):
            node.set('endLine', '[]')
        roads.append({'id': eid, 'name': node.get('name'), 'from': a, 'to': b,
                      'points': pts, 'elevations': [p[2] for p in xyz],
                      'full_points': [junctions[a]['position']]+pts+[junctions[b]['position']],
                      'width': node.get('width'), 'number_of_lanes': int(node.get('numberOfLanes')),
                      'forward_lanes': int(node.get('numberOfForwardLanes')), 'training': eid == 'NORTH_TRAINING'})
    by_id = {n.get('id'): n for n in nodes if n.get('id')}
    extras = []
    for nid, n in junctions.items():
        n['world_id'] = native_id(nid)
        if n['kind'] == 'roundabout_port':
            continue
        vertices = [p for port in n['ports'] for p in mouth(port['position'], port['outward'], port['width'])]
        polygon = g.hull(vertices)
        n['polygon'] = polygon if n['kind'] != 'connection' else []
        if native_id(nid) not in by_id:
            by_id[native_id(nid)] = Node('Crossroad', 'Crossroad {\n  name "Junction-'+nid+'"\n  id "'+native_id(nid)+'"\n  translation 0 0 0.008\n  appearance '+g.ASPHALT+'\n  shape []\n}')
            extras.append(by_id[native_id(nid)])
        node = by_id[native_id(nid)]
        node.set('connectedRoadIDs', json.dumps([p['edge'] for p in n['ports']]))
        if node.type == 'Crossroad' and n['kind'] != 'connection':
            node.set('shape', '[\n    '+'\n    '.join(g.vec((-y, -x, 0)) for x, y in polygon)+'\n  ]')
        if n['kind'] != 'connection':
            extras.append(Node('Solid', curb_node(nid, n)))
    by_id['ROUNDABOUT'].set('connectedRoadIDs', json.dumps([p['edge'] for nid, n in junctions.items() if nid.startswith('R_') for p in n['ports']]))
    # Replace only previously generated curb groups on repeated synchronization.
    for n in nodes:
        if n.get('name', '').startswith('JunctionCurbs-'):
            n.text = ''
    g.edges[:] = roads
    g.junctions.clear()
    g.junctions.update(junctions)
    barrier = next(n for n in nodes if n.type == 'CrashBarrier')
    if 'expansion' in data:
        by_edge = {e['id']: e for e in roads}
        outline_points = [p for eid in data['expansion']['perimeter'] for p in by_edge[eid]['points']]
        outline = Polygon(outline_points).buffer(1.45, join_style=2)
        assert outline.is_valid and outline.geom_type == 'Polygon'
        border = list(outline.exterior.coords)
        barrier.set('path', '[ '+' '.join(g.vec((*p, road_height(p, roads)-.008)) for p in border)+' ]')
    else:
        g.draw_outer_barrier()
        barrier.set('path', re.search(r'path (\[[^]]*\])', g.chunks[-1])[1])
    world = re.sub(r'\n{3,}', '\n\n', rewrite(source, nodes).rstrip()) + '\n\n' + '\n\n'.join(n.text.strip() for n in extras) + '\n'
    g.lane_graph.clear()
    g.chunks.clear()
    for road in roads:
        g.draw_edge(road)  # Reuse routing geometry; emitted road text is discarded.
    ring = data['roundabout']
    for index in range(1, ring['ring_lanes']+1):
        for i, nid in enumerate(['R_E', 'R_N', 'R_W', 'R_S']):
            nxt = ['R_E', 'R_N', 'R_W', 'R_S'][(i+1) % 4]
            g.lane_graph[f'RING_{nid}_{index}'] = {'from': nid, 'to': nxt,
                'points': g.arc(ring['center'], ring['inner_radius']+(ring['outer_radius']-ring['inner_radius'])/ring['ring_lanes']*(index-.5), i*math.pi/2, (i+1)*math.pi/2),
                'lane_index': index, 'successors': [], 'speed_limit_m_s': .25}
    g.connectors()
    for lane in g.lane_graph.values():
        support = [e for e in roads if e['id'] == lane.get('road_id')]
        lane['elevations'] = [road_height(p, support) if support else .008 for p in lane['points']]
    data.update(roads=roads, junctions=junctions, lanes=g.lane_graph)
    path.write_text(world, encoding='utf-8')
    (ROOT/'map.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    g.overview(data['bounds'],data['roundabout'])
    print(f'Synchronized {len(roads)} roads, {len(junctions)} junctions/ports, {len(g.lane_graph)} lanes/connectors.')


def road_height(point, roads):
    best, height = math.inf, .008
    for road in roads:
        for i, (a, b) in enumerate(zip(road['points'], road['points'][1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0, min(1, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
            distance = (point[0]-a[0]-t*dx)**2+(point[1]-a[1]-t*dy)**2
            if distance < best:
                best = distance
                height = road['elevations'][i]*(1-t)+road['elevations'][i+1]*t
    return height


if __name__ == '__main__':
    main()
