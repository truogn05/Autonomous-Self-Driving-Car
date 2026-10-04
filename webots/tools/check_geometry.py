"""Check generated lane footprints against the road union (requires Shapely for QA only)."""
from pathlib import Path
import json
import math
import hashlib
import re
from shapely.geometry import LineString, Polygon, Point
from shapely.ops import unary_union

root=Path(__file__).resolve().parents[1]
data=json.loads((root/'map.json').read_text(encoding='utf-8'))
world=(root/'worlds/my_world.wbt').read_text(encoding='utf-8')
assert not re.search(r'\bsolid\s+(?:TRUE|FALSE)',world), 'IndexedFaceSet does not support solid in Webots R2025a'
solid_headers=re.findall(r'\bSolid\s*\{([^{}]*)',world)
solid_names=[]
for header in solid_headers:
    match=re.search(r'\bname\s+"([^"]+)"',header)
    assert match, 'Unnamed Solid would default to solid and collide with siblings'
    solid_names.append(match.group(1))
assert len(solid_names)==len(set(solid_names)), 'Duplicate Solid names'
assert sum(name.startswith('StopSign-') for name in solid_names)==5
assert sum(name.startswith('TrafficLightPole-') for name in solid_names)==4
assert sum(name.startswith('TrafficLightHousing-') for name in solid_names)==4
assert not re.search(r'DEF\s+(?:SIGN_|YIELD_|DIRECTION_|SPEED_LIMIT)',world), 'Signs should display their name, not DEF'
assert not re.search(r'^Shape\s*\{',world,re.M), 'Manual road-paint Shapes remain'
assert not re.search(r'DEF\s+\w+\s+(?:Road|StraightRoadSegment|Crossroad|RoadIntersection|Roundabout)\s*\{',world), 'Road DEF names remain'
assert len(re.findall(r'numberOfLanes 4 numberOfForwardLanes 2',world)) == len(data['roads'])
assert all(r['number_of_lanes']==4 and r['forward_lanes']==2 for r in data['roads'])
assert all(r['width']==2.4 and r['name']!=r['id'] for r in data['roads'])
proto_dir=root/'protos/vendor/projects/objects/road/protos'
for field in re.findall(r'(?:startLine|endLine)\s*\[([^]]*)\]',world):
    for url in re.findall(r'"([^"]+)"',field):
        assert (proto_dir/url).resolve().is_file(), url
scale=data.get('road_scale',1)
surfaces=[]
for road in data['roads']:
    if 'drivable_polygon' in road:
        surfaces.append(Polygon(road['drivable_polygon']))
    else:
        surfaces.append(LineString(road['points']).buffer(road['width']/2,cap_style=2,join_style=2))
for n in data['junctions'].values():
    if 'polygon' in n:
        surfaces.append(Polygon(n['polygon']))
surfaces.append(Point(6*scale,0).buffer(2*scale,resolution=128).difference(Point(6*scale,0).buffer(1.2*scale,resolution=128)))
for a,b in [((8,0),(8.6,0)),((6,2),(6,2.6)),((4,0),(3.4,0)),((6,-2),(6,-2.6))]:
    surfaces.append(LineString([(x*scale,y*scale) for x,y in [a,b]]).buffer(.6*scale,cap_style=2))
# Roundabout.proto fills the small gaps between the circular edge and each
# tangent road with two triangle fans (128 subdivisions for this world).
theta=math.atan2(.6,2)
theta2=math.atan2(.6+5*(.6-2*math.sin(theta)),2)
steps=round(theta2/(math.tau/128))
for angle in [0,math.pi/2,math.pi,3*math.pi/2]:
    for side in [-1,1]:
        mouth=(6+2*math.cos(angle)-side*.6*math.sin(angle),2*math.sin(angle)+side*.6*math.cos(angle))
        curve=[(6+2*math.cos(angle+side*j*math.tau/128),2*math.sin(angle+side*j*math.tau/128)) for j in range(steps+1)]
        surfaces.append(Polygon([(x*scale,y*scale) for x,y in [mouth]+curve]))
surface=unary_union(surfaces).buffer(.003)
failures=[]
for lid,lane in data['lanes'].items():
    route=LineString(lane['points'])
    swept=route.buffer(.186,cap_style=1,join_style=1)
    outside=swept.difference(surface)
    if outside.area>.0001:
        failures.append({'lane':lid,'outside_m2':round(outside.area,6),'bounds':list(outside.bounds)})
report={'footprint_radius_m':.186,'lanes_checked':len(data['lanes']),'outside_road':failures,
        'world_sha256':hashlib.sha256((root/'worlds/my_world.wbt').read_bytes()).hexdigest(),
        'map_sha256':hashlib.sha256((root/'map.json').read_bytes()).hexdigest(),
        'note':'Geometry QA, not a vehicle dynamics or localization test. Stationary training obstacle is intentional.'}
(root/'validation').mkdir(exist_ok=True)
(root/'validation/geometry.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
raise SystemExit(bool(failures))
