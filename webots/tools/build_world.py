"""Build the reference-inspired traffic practice town (metres, ENU, right-hand traffic).

One geometry definition produces the WBT, lane centre lines and an SVG overview.
Run from any directory: python webots/tools/build_world.py
"""
from pathlib import Path
import json
import math
import shutil
from shapely.geometry import Polygon

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'worlds'
TAU = math.tau
Z = 0.008
ASPHALT = 'PBRAppearance { baseColor 0.12 0.14 0.16 roughness 1 metalness 0 }'
PAVEMENT = 'PBRAppearance { baseColor 0.65 0.67 0.62 roughness 1 metalness 0 }'
WHITE = '0.96 0.95 0.87'
YELLOW = '1 0.72 0.12'
SCALE = 2
ROAD_WIDTH = 2.4
LANE_WIDTH = .6
ROAD_LINES = '''[
  RoadLine { type "dashed" color 1 1 1 width .025 }
  RoadLine { type "double" color .85 .75 .3 width .025 }
  RoadLine { type "dashed" color 1 1 1 width .025 }
]'''
NAME_COUNTS = {}


def next_name(kind):
    NAME_COUNTS[kind] = NAME_COUNTS.get(kind, 0) + 1
    return f'{kind}-{NAME_COUNTS[kind]}'


chunks, edges, junctions, lane_graph = [], [], {}, {}


def fmt(x):
    return f'{x:.6f}'.rstrip('0').rstrip('.') or '0'


def vec(p):
    return ' '.join(fmt(v) for v in p)


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def mul(a, s):
    return (a[0] * s, a[1] * s)


def direction(a, b):
    d = math.dist(a, b)
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def normal(u):
    return (-u[1], u[0])


def emit(s):
    chunks.append(s)


def box(name, p, size, color, angle=0, collision=True):
    emit(f'''DEF {name} Solid {{ translation {vec(p)} rotation 0 0 1 {fmt(angle)}
      name "{name}" children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 0.85 metalness 0 }}
      geometry Box {{ size {vec(size)} }} }} ]
      {'boundingObject Box { size ' + vec(size) + ' }' if collision else ''} }}''')


def arc(center, r, start, end, count=24):
    return [(center[0] + r * math.cos(start + (end-start)*i/count),
             center[1] + r * math.sin(start + (end-start)*i/count)) for i in range(count+1)]


def bezier(a, b, c, d, count=32):
    return [tuple((1-t)**3*a[k] + 3*(1-t)**2*t*b[k] + 3*(1-t)*t*t*c[k] + t**3*d[k]
                  for k in range(2)) for t in [i/count for i in range(count+1)]]


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def at(points, s):
    for a, b in zip(points, points[1:]):
        d = math.dist(a, b)
        if s <= d:
            return add(a, mul(direction(a, b), max(0, s)))
        s -= d
    return points[-1]


def cut(points, start, end):
    total = length(points)
    kept, acc = [at(points, start)], 0
    for a, b in zip(points, points[1:]):
        acc += math.dist(a, b)
        if start + 1e-5 < acc < total-end-1e-5:
            kept.append(b)
    kept.append(at(points, total-end))
    return kept


def offset(points, distance):
    result = []
    for i, p in enumerate(points):
        u = direction(points[max(0, i-1)], points[min(len(points)-1, i+1)])
        result.append(add(p, mul(normal(u), distance)))
    return result


def hull(points):
    pts = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo)>1 and cross(lo[-2], lo[-1], p)<=0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi)>1 and cross(hi[-2], hi[-1], p)<=0:
            hi.pop()
        hi.append(p)
    return lo[:-1]+hi[:-1]


def node(name, x, y, kind='T'):
    junctions[name] = {'position': [x,y], 'kind': kind, 'ports': [], 'trim': 0 if kind=='roundabout_port' else 0.8}


def edge(name, a, b, points=None, training=False):
    points = points or [junctions[a]['position'], junctions[b]['position']]
    assert math.dist(points[0], junctions[a]['position']) < 1e-6
    assert math.dist(points[-1], junctions[b]['position']) < 1e-6
    trimmed = cut(points, junctions[a]['trim'], junctions[b]['trim'])
    assert length(trimmed) > .3, name
    e = {'id':name, 'from':a, 'to':b, 'points':trimmed, 'full_points':points, 'width':1.2, 'training':training}
    edges.append(e)
    for nid, p, u in [(a, trimmed[0], direction(trimmed[0], trimmed[1])),
                       (b, trimmed[-1], direction(trimmed[-1], trimmed[-2]))]:
        junctions[nid]['ports'].append({'edge':name, 'position':p, 'outward':u})


def make_network():
    for n,x,y,k in [
        ('WEST_UPPER',-10.5,3.5,'T'),('WEST_MIDDLE',-10.5,0,'T'),('WEST_LOWER',-10.5,-3.3,'T'),
        ('NORTH_WEST',-4,7.5,'T'),('NORTH_MIDDLE',1.5,7.5,'T'),('NORTH_EAST',6,7.5,'T'),
        ('EAST',10.5,0,'T'),('LOWER_EAST',6,-6.5,'T'),
        ('BLOCK_NORTH',-7,3.5,'T'),('BLOCK_CROSS',-7,0,'X'),('BLOCK_SOUTH',-7,-3.3,'T'),
        ('INNER_NORTH',-4,3.5,'X'),('SIGNAL_CROSS',-4,0,'X'),('INNER_SOUTH',-4,-3.3,'T'),
        ('MID_BEND',-.5,0,'T'),('MID_AXIS',1.5,0,'T'),('Y_SOUTH',-3.5,-5.8,'Y'),
        ('R_W',3.4,0,'roundabout_port'),('R_N',6,2.6,'roundabout_port'),
        ('R_E',8.6,0,'roundabout_port'),('R_S',6,-2.6,'roundabout_port')]:
        node(n,x,y,k)
    edge('OUTER_NW','WEST_UPPER','NORTH_WEST', [(-10.5,3.5),(-10.5,6)]+arc((-9,6),1.5,math.pi,math.pi/2)[1:]+[(-4,7.5)])
    edge('NORTH_TRAINING','NORTH_WEST','NORTH_MIDDLE',training=True)
    edge('NORTH_LINK','NORTH_MIDDLE','NORTH_EAST')
    edge('OUTER_NE','NORTH_EAST','EAST',[(6,7.5),(9,7.5)]+arc((9,6),1.5,math.pi/2,0)[1:]+[(10.5,0)])
    edge('OUTER_SE','EAST','LOWER_EAST',[(10.5,0),(10.5,-6)]+arc((9,-6),1.5,0,-math.pi/2)[1:]+
         bezier((9,-7.5),(7.8,-7.5),(7.2,-6.5),(6,-6.5))[1:])
    edge('S_CURVES','Y_SOUTH','LOWER_EAST',bezier((-3.5,-5.8),(-1.4,-5.8),(-.7,-5.0),(.7,-6.3))+
         bezier((.7,-6.3),(2.5,-8),(3.8,-6.5),(6,-6.5))[1:])
    edge('OUTER_SW','WEST_LOWER','Y_SOUTH',[(-10.5,-3.3),(-10.5,-6)]+arc((-9,-6),1.5,math.pi,1.5*math.pi)[1:]+
         bezier((-9,-7.5),(-6.7,-7.5),(-5.7,-7.5),(-3.5,-5.8))[1:])
    for name,a,b in [('WEST_1','WEST_UPPER','WEST_MIDDLE'),('WEST_2','WEST_MIDDLE','WEST_LOWER'),
        ('BLOCK_TOP_W','WEST_UPPER','BLOCK_NORTH'),('BLOCK_TOP_E','BLOCK_NORTH','INNER_NORTH'),
        ('BLOCK_MID_W','WEST_MIDDLE','BLOCK_CROSS'),('BLOCK_MID_E','BLOCK_CROSS','SIGNAL_CROSS'),
        ('BLOCK_BOTTOM_W','WEST_LOWER','BLOCK_SOUTH'),('BLOCK_BOTTOM_E','BLOCK_SOUTH','INNER_SOUTH'),
        ('RECONNECTED_NORTH','BLOCK_NORTH','BLOCK_CROSS'),('RECONNECTED_SOUTH','BLOCK_CROSS','BLOCK_SOUTH'),
        ('INNER_1','NORTH_WEST','INNER_NORTH'),('INNER_2','INNER_NORTH','SIGNAL_CROSS'),
        ('INNER_3','SIGNAL_CROSS','INNER_SOUTH'),('CENTRAL_1','SIGNAL_CROSS','MID_BEND'),
        ('CENTRAL_2','MID_BEND','MID_AXIS'),('CENTRAL_3','MID_AXIS','R_W'),
        ('CENTRAL_AXIS','MID_AXIS','NORTH_MIDDLE'),('ROUNDABOUT_NORTH','R_N','NORTH_EAST'),
        ('ROUNDABOUT_EAST','R_E','EAST'),('ROUNDABOUT_SOUTH','R_S','LOWER_EAST')]:
        edge(name,a,b)
    edge('Y_STEM','INNER_SOUTH','Y_SOUTH',[(-4,-3.3),(-4,-4.3),(-3.5,-5.8)])
    edge('INNER_CURVE','INNER_NORTH','MID_BEND',[(-4,3.5),(-2,3.5)]+arc((-2,2),1.5,math.pi/2,0)[1:]+[(-.5,0)])


def crossroad(name, polygon):
    # Upstream Crossroad uses a legacy internal rotation: (x,y,z) -> (-y,-x,-z).
    kind = 'YJunction' if 'Y_SOUTH' in name else ('Crossroad' if junctions[name[2:]]['kind']=='X' else 'TJunction')
    label = next_name(kind)
    emit(f'''Crossroad {{ name "{label}" id "{name}"
        translation 0 0 {Z} shape [ {' '.join(vec((-y,-x,0)) for x,y in polygon)} ]
        appearance {ASPHALT} boundingObject FALSE }}''')


def scale_network():
    for n in junctions.values():
        n['position'] = mul(n['position'], SCALE)
        n['trim'] *= SCALE
        for port in n['ports']:
            port['position'] = mul(port['position'], SCALE)
    for e in edges:
        e['points'] = [mul(p, SCALE) for p in e['points']]
        e['full_points'] = [mul(p, SCALE) for p in e['full_points']]
        e['width'] = ROAD_WIDTH
        e['number_of_lanes'] = 4
        e['forward_lanes'] = 2
        e['name'] = next_name('StraightRoad' if len(e['points']) == 2 else 'Road')


def end_markings(nid, start):
    # Road.proto resolves texture fields relative to itself, not to the WBT.
    prefix = '../../../../../../worlds/textures/'
    controlled = junctions[nid]['kind'] in ('T', 'X', 'Y')
    crossing = nid in ('SIGNAL_CROSS','WEST_MIDDLE','NORTH_MIDDLE')
    if not controlled and not crossing:
        return '[]'
    slots = []
    for i in range(4):
        incoming = i >= 2 if start else i < 2
        paint = ('stop_crossing' if crossing else 'stop_line') if incoming else ('crossing' if crossing else None)
        slots.append(prefix + paint + '.png' if paint else '')
    return '[ ' + ' '.join('"'+v+'"' for v in slots) + ' ]'


def draw_outer_barrier():
    # Clockwise perimeter, including the outside of each junction. No barriers
    # cross the internal access roads. Keep the standard CrashBarrier profile,
    # lowered so its collider spans z=.02.. .42 for the small robot.
    perimeter = [('OUTER_NW', False), ('NORTH_TRAINING', False), ('NORTH_LINK', False),
                 ('OUTER_NE', False), ('OUTER_SE', False), ('S_CURVES', True),
                 ('OUTER_SW', True), ('WEST_2', True), ('WEST_1', True)]
    by_id = {e['id']: e for e in edges}
    points = []
    for eid, reverse in perimeter:
        # Bridge the trimmed road ends across the outside of a junction;
        # following its centre creates a cusp at the southern Y junction.
        seq = by_id[eid]['points']
        seq = list(reversed(seq)) if reverse else seq
        for p in seq:
            if not points or math.dist(points[-1], p) > 1e-6:
                points.append(p)
    # A polygon offset removes local loops at concave curve/junction joins.
    # Reuse Shapely (also used by check_geometry), rather than hand-rolling it.
    outline = Polygon(points).buffer(ROAD_WIDTH/2 + .25, join_style=2)
    assert outline.geom_type == 'Polygon' and outline.is_valid
    path = list(outline.exterior.coords)
    emit(f'''CrashBarrier {{ name "OuterBarrier-1" translation 0 0 -.38
      poleGap .6 barrierOnRightSide FALSE enableBoundingObject TRUE subdivision -1
      path [ {' '.join(vec((*p, 0)) for p in path)} ] }}''')



def draw_edge(e):
    pts, name = e['points'], e['id']
    common=f'''name "{e['name']}" id "{name}" width {ROAD_WIDTH}
      numberOfLanes 4 numberOfForwardLanes 2 lines {ROAD_LINES}
      startLine {end_markings(e['from'], True)} endLine {end_markings(e['to'], False)}
      appearance {ASPHALT} pavementAppearance {PAVEMENT}
      roadBorderHeight 0.035 roadBoundingObject FALSE
      rightBorderBoundingObject TRUE leftBorderBoundingObject TRUE'''
    if len(pts)==2:
        angle=math.atan2(pts[-1][1]-pts[0][1],pts[-1][0]-pts[0][0])
        emit(f'''StraightRoadSegment {{ translation {vec((*pts[0],Z))}
          rotation 0 0 1 {angle} length {length(pts)} startingRoadBorderWidth .14 endingRoadBorderWidth .14 {common} }}''')
    else:
        emit(f'''Road {{ translation 0 0 {Z} wayPoints [ {' '.join(vec((*p,0)) for p in pts)} ]
          splineSubdivision -1 roadBorderWidth [ .14 ] {common} }}''')
    for reverse in (False,True):
        seq=list(reversed(pts)) if reverse else pts
        for index in (1,2):
            lid=name+('_B' if reverse else '_F')+str(index)
            lane_graph[lid]={'from':e['to'] if reverse else e['from'], 'to':e['from'] if reverse else e['to'],
                'points':offset(seq,-LANE_WIDTH*(index-.5)), 'lane_index':index,
                'road_id': e['id'], 'successors':[], 'speed_limit_m_s':.5}
    # Split at a shared on-road change zone; dashed dividers permit changing
    # lane before selecting a junction turn, rather than switching at the junction.
    for suffix in ('F','B'):
        zone='CHANGE_'+name+'_'+suffix
        for index in (1,2):
            lid=name+'_'+suffix+str(index)
            lane=lane_graph.pop(lid)
            points=lane['points']; total=length(points); gap=min(.8,total/4)
            lane_graph[lid+'_entry']={**lane,'successors':[],'to':zone,'points':cut(points,0,total/2+gap)}
            lane_graph[lid+'_exit']={**lane,'successors':[],'from':zone,'points':cut(points,total/2+gap,0)}


def draw_junctions():
    for name,n in junctions.items():
        if n['kind']=='roundabout_port':
            continue
        vertices=[]
        for port in n['ports']:
            side=mul(normal(port['outward']),ROAD_WIDTH/2)
            vertices.extend([add(port['position'],side),add(port['position'],mul(side,-1))])
        polygon=hull(vertices)
        n['polygon']=polygon
        # The signal crossing is aligned to the cardinal axes. RoadIntersection's
        # default sockets are diagonal; rotating 45 degrees aligns them.
        if name=='SIGNAL_CROSS':
            emit(f'''RoadIntersection {{ translation {vec((*n['position'],Z))} rotation 0 0 1 {math.pi/4}
               name "RoadIntersection-1" id "{name}" roadNumber 4 roadsWidth 2.4 startRoads TRUE
               startRoadsLength [ .4 ] startRoadBorder FALSE startRoadsNumberOfLanes 4 startRoadsLine {ROAD_LINES}
               startRoadsStartLine [] startRoadsEndLine [] appearance {ASPHALT} }}''')
        else:
            crossroad('J_'+name,polygon)
    emit(f'''Roundabout {{ translation 12 0 {Z} rotation 0 0 1 {-math.pi/4}
      name "Roundabout-1" id "ROUNDABOUT" innerRadius 2.4 outerRadius 4 roadNumber 4
      subdivision 32 startRoadsLength [ 1.2 ] startRoadsWidth 2.4 startRoadsNumberOfLanes 4
      startRoadsLine {ROAD_LINES}
      startRoadsStartLine [] startRoadsEndLine [] borderWidth .10 borderHeight .04
      roadBoundingObject FALSE borderBoundingObject TRUE centerBoundingObject TRUE appearance {ASPHALT} }}''')
    # Explicit CCW circulating lanes and entry/exit connectors, rather than
    # treating the roundabout as a free-for-all junction in the routing graph.
    for i,nid in enumerate(['R_E','R_N','R_W','R_S']):
        nxt=['R_E','R_N','R_W','R_S'][(i+1)%4]
        lane_graph['RING_'+nid]={'from':nid,'to':nxt,'points':arc((12,0),3.2,i*math.pi/2,(i+1)*math.pi/2),
                                'successors':[],'speed_limit_m_s':.25}


def sign(name,p,facing,kind):
    # Miniature boards keep the pole and collider at the same physical scale.
    theta=math.atan2(facing[1],facing[0])
    label = next_name({'stop':'StopSign','yield':'YieldSign','roundabout':'RoundaboutSign','speed':'SpeedLimitSign'}[kind])
    emit(f'''Solid {{ translation {vec((*p,0))} rotation 0 0 1 {theta} name "{label}" description "{name}"
       children [ Pose {{ translation 0 0 .28 children [ Shape {{ appearance PBRAppearance {{ baseColor .55 .58 .60 metalness .5 roughness .5 }}
          geometry Cylinder {{ radius .012 height .56 }} }} ] }}
       Pose {{ translation .016 0 .51 children [ Shape {{ appearance PBRAppearance {{
          baseColorMap ImageTexture {{ url [ "textures/{kind}.png" ] }} roughness 1 metalness 0 }}
          geometry IndexedFaceSet {{ coord Coordinate {{ point [ 0 -.13 -.13, 0 .13 -.13, 0 .13 .13, 0 -.13 .13 ] }}
          texCoord TextureCoordinate {{ point [0 0,1 0,1 1,0 1] }} coordIndex [0 1 2 3 -1] }} }} ] }} ]
       boundingObject Pose {{ translation 0 0 .28 children [ Cylinder {{ radius .015 height .56 }} ] }} }}''')


def furniture():
    for name in ['WEST_UPPER','NORTH_MIDDLE','NORTH_EAST','BLOCK_NORTH','BLOCK_SOUTH','INNER_SOUTH','Y_SOUTH']:
        n=junctions[name]
        # Select one minor approach; no all-way-stop clutter.
        port=n['ports'][-1]
        u=port['outward']; p=add(port['position'],add(mul(u,.48),mul(normal(u),1.39)))
        sign('SIGN_'+name,p,u,'yield' if name in ['Y_SOUTH','NORTH_EAST'] else 'stop')
    for nid in ['R_E','R_N','R_W','R_S']:
        n=junctions[nid]; u=direction((12,0),n['position']); p=n['position']
        sign('YIELD_'+nid,add(p,add(mul(u,.25),mul(normal(u),1.4))),u,'yield')
        sign('DIRECTION_'+nid,add(p,add(mul(u,.55),mul(normal(u),1.4))),u,'roundabout')
    sign('SPEED_LIMIT',(-17.6,13.5),(0,-1),'speed')
    emit('DEF SIGNAL_SYSTEM Robot { name "junction signals" controller "traffic_signals" children [')
    for i,port in enumerate(junctions['SIGNAL_CROSS']['ports']):
        p,u=port['position'],port['outward']; n=normal(u)
        pos=add(p,add(mul(u,.05),mul(n,1.42)))
        group='EW' if abs(u[0])>.5 else 'NS'
        emit(f'Pose {{ translation {vec((*pos,0))} rotation 0 0 1 {math.atan2(u[1],u[0])} children [')
        emit('Solid { name "TrafficLightPole-%d" translation 0 0 .40 children [ Shape { appearance PBRAppearance { baseColor .3 .33 .35 metalness .5 roughness .5 } geometry Cylinder { radius .015 height .8 } } ] boundingObject Cylinder { radius .015 height .8 } }' % (i+1))
        emit('Solid { name "TrafficLightHousing-%d" translation 0 0 .78 children [ Shape { appearance PBRAppearance { baseColor .035 .04 .045 roughness 1 metalness 0 } geometry Box { size .10 .15 .34 } } ] boundingObject Box { size .10 .15 .34 } }' % (i+1))
        for j,(color,rgb) in enumerate([('red','1 0 0'),('amber','1 .55 0'),('green','0 1 .18')]):
            emit(f'''LED {{ translation .062 0 {fmt(.88-j*.1)} name "{group}_{i}_{color}" color [ {rgb} ]
              children [ Shape {{ appearance PBRAppearance {{ baseColor .08 .08 .08 roughness .5 metalness 0 emissiveIntensity 3 }}
              geometry Sphere {{ radius .039 subdivision 2 }} }} ] }}''')
        emit('] }')
    emit('] }')
    box('TRAINING_OBSTACLE',(-2.5,14.1,.10),(.16,.06,.20),'0.95 0.35 0.06')
    # Small parked markers are outside the roadway; no decorative obstacle on a lane.
    box('START_MARKER',(-19.5,-10.6,.012),(.3,.5,.018),'0.12 .65 .95',collision=False)
    box('GOAL_MARKER',(19.5,6,.012),(.3,.5,.018),'.95 .6 .1',collision=False)


def connectors():
    original=list(lane_graph.items())
    for lid,lane in original:
        nid=lane['to']
        for other,target in original:
            if target['from']!=nid:
                continue
            # No automatic U-turn at ordinary intersections or roundabout entries.
            if not nid.startswith('CHANGE_') and lane.get('road_id') and lane.get('road_id') == target.get('road_id'):
                continue
            if nid.startswith('R_') and not (lid.startswith('RING_') or other.startswith('RING_')):
                continue
            if nid.startswith('R_') and lane.get('lane_index') != target.get('lane_index'):
                continue
            if not nid.startswith(('CHANGE_', 'R_')) and 'lane_index' in lane and 'lane_index' in target:
                incoming=direction(lane['points'][-2],lane['points'][-1])
                outgoing=direction(target['points'][0],target['points'][1])
                turn=math.atan2(incoming[0]*outgoing[1]-incoming[1]*outgoing[0],incoming[0]*outgoing[0]+incoming[1]*outgoing[1])
                if abs(turn)<.45:
                    if lane['lane_index']!=target['lane_index']:
                        continue
                elif lane['lane_index'] != (1 if turn>0 else 2) or target['lane_index']!=lane['lane_index']:
                    continue
            a,d=lane['points'][-1],target['points'][0]
            if math.dist(a,d)<.001:
                lane['successors'].append(other)
                continue
            u=direction(lane['points'][-2],a)
            v=direction(d,target['points'][1])
            # Native RoadIntersection has square inside corners; turn farther
            # into the intersection so the robot footprint clears those corners.
            reach=math.dist(a,d)*(.6 if nid=='SIGNAL_CROSS' else .45)
            cid=f'{lid}__{other}'
            lane_graph[cid]={'from':nid,'to':nid,'points':bezier(a,add(a,mul(u,reach)),add(d,mul(v,-reach)),d,16),
                             'successors':[other],'speed_limit_m_s':.25,'junction':nid}
            lane['successors'].append(cid)


def overview(bounds=(-24,24,-18,18)):
    xmin,xmax,ymin,ymax=bounds
    width,height=(xmax-xmin)*22.5+40,(ymax-ymin)*22.5+40
    def path(pts):
        return ' '.join(('M' if i==0 else 'L')+f'{(x-xmin)*22.5+20:.2f},{20+(ymax-y)*22.5:.2f}' for i,(x,y) in enumerate(pts))
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
         '<rect width="100%" height="100%" fill="#e9eddf"/>']
    for e in edges:
        svg.append(f'<path d="{path(e["full_points"])}" fill="none" stroke="#343c42" stroke-width="54" stroke-linejoin="round"/>')
    svg.append(f'<circle cx="{(12-xmin)*22.5+20}" cy="{20+ymax*22.5}" r="72" fill="none" stroke="#343c42" stroke-width="36"/>')
    for e in edges:
        svg.append(f'<path d="{path(e["points"])}" fill="none" stroke="#edc546" stroke-width="1.2"/>')
    for name,n in junctions.items():
        x,y=n['position']; x=(x-xmin)*22.5+20;y=20+(ymax-y)*22.5
        svg.append(f'<text x="{x+5}" y="{y-14}" font-family="Arial" font-size="10" fill="#b51f36" stroke="white" stroke-width="2" paint-order="stroke">{name}</text>')
    svg.append(f'<text x="25" y="32" font-family="Arial" font-size="21" fill="#263c34">MY WORLD / traffic practice town / {xmax-xmin} x {ymax-ymin} m</text></svg>')
    (ROOT/'map.svg').write_text('\n'.join(svg),encoding='utf-8')


def validate():
    # Verify the real generated directed network, not only the undirected sketch.
    assert all(len(n['ports'])>=3 for n in junctions.values() if n['kind']!='roundabout_port')
    assert sum(n['kind']=='Y' for n in junctions.values())==1
    for lid,lane in lane_graph.items():
        assert lane['successors'], lid
        for nxt in lane['successors']:
            assert math.dist(lane['points'][-1],lane_graph[nxt]['points'][0])<.001,(lid,nxt)
    for reverse in (False,True):
        graph={k:[] for k in lane_graph}
        for k,v in lane_graph.items():
            for nxt in v['successors']:
                graph[nxt if reverse else k].append(k if reverse else nxt)
        visited=set(); todo=[next(iter(graph))]
        while todo:
            p=todo.pop()
            if p not in visited:
                visited.add(p);todo.extend(graph[p])
        assert len(visited)==len(graph), 'Directed lanes are not strongly connected'
    for lane in lane_graph.values():
        for x,y in lane['points']:
            assert -24<x<24 and -18<y<18


def legacy_flat_world():
    OUT.mkdir(exist_ok=True)
    world=OUT/'my_world.wbt'
    backup=OUT/'my_world.original.wbt'
    if world.exists() and not backup.exists():
        shutil.copy2(world,backup)
    emit('#VRML_SIM R2025a utf8\n# Generated by ../tools/build_world.py. Dimensions in metres; ENU.')
    for proto in ['StraightRoadSegment','Road','RoadLine','Crossroad','RoadIntersection','Roundabout','CrashBarrier']:
        emit(f'EXTERNPROTO "../protos/vendor/projects/objects/road/protos/{proto}.proto"')
    emit('''WorldInfo { title "My World - connected traffic practice town" basicTimeStep 32 coordinateSystem "ENU" randomSeed 7 }
    DEF OVERVIEW Viewpoint { orientation -.577350269 .577350269 .577350269 2.094395102 position 0 0 86 fieldOfView 0.85 description "Town overview - north up" }
    Background { skyColor [ .72 .82 .90 ] }
    DirectionalLight { direction -.4 -.6 -1 intensity 1.2 ambientIntensity .65 castShadows TRUE }''')
    box('GROUND',(0,0,-.06),(48,36,.12),'.38 .48 .32')
    make_network()
    scale_network()
    for e in edges:
        draw_edge(e)
    draw_junctions()
    draw_outer_barrier()
    furniture()
    connectors()
    validate()
    emit('DEF INSPECTOR Robot { supervisor TRUE name "world inspector" controller "world_inspector" }')
    world.write_text('\n\n'.join(chunks)+'\n',encoding='utf-8')
    data={'units':'metres','coordinate_system':'ENU','traffic_side':'right','bounds':[-24,24,-18,18], 'road_scale':SCALE, 'roundabout':{'center':[12,0],'inner_radius':2.4,'outer_radius':4,'ring_lanes':1},
          'spawn':{'position':[-20.1,-10.6,.10],'heading':math.pi/2},
          'goal':{'position':[20.1,6,.10],'heading':-math.pi/2},
          'junctions':junctions,'roads':edges,'lanes':lane_graph,
          'training':{'road':'NORTH_TRAINING','obstacle_def':'TRAINING_OBSTACLE','lane_width':.6,'lanes_per_direction':2}}
    (ROOT/'map.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    overview()
    print(f'Built {world}: {len(edges)} roads, {len(junctions)} junctions/ports, {len(lane_graph)} directed lanes/connectors.')


if __name__=='__main__':
    # The saved WBT includes GUI edits and the tunnel; synchronize it in place.
    from sync_world import main
    main()
