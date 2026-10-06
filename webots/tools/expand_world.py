"""Enlarge the saved town once, keeping its tunnel and reconnecting every road."""
import json
import math
import re
import shutil
import tempfile
from pathlib import Path
import build_world as g
import sync_world as s

ROOT = s.ROOT
MIDDLE = ['BLOCK_MID_TUNNEL', 'CENTRAL_1', 'CENTRAL_2', 'CENTRAL_3', 'ROUNDABOUT_EAST']


def lines(count):
    entries = ['RoadLine { width .025'+(' color .85 .75 .3 type "double"' if i == count//2-1 else '')+' }' for i in range(count-1)]
    return '[ '+' '.join(entries)+' ]'


def trim3(points, start, end):
    total = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
    assert total > start+end+.3, ('Road is too short for its junctions', total, start, end)
    def at(distance):
        for a, b in zip(points, points[1:]):
            span = math.dist(a[:2], b[:2])
            if distance <= span+1e-9:
                t = max(0, min(1, distance/span))
                return [a[k]*(1-t)+b[k]*t for k in range(3)]
            distance -= span
        return points[-1]
    result, distance = [at(start)], 0
    for a, b in zip(points, points[1:]):
        distance += math.dist(a[:2], b[:2])
        if start+1e-5 < distance < total-end-1e-5:
            result.append(b)
    result.append(at(total-end))
    return resample(result) if len(result)>2 else [[round(v, 6) for v in p] for p in result]


def resample(points):
    spans = [math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])]
    total = sum(spans)
    count = math.ceil(total/.3)
    result, index, accumulated = [], 0, 0
    for i in range(count+1):
        distance = total*i/count
        while index<len(spans)-1 and accumulated+spans[index]<distance-1e-9:
            accumulated += spans[index];index += 1
        t=max(0,min(1,(distance-accumulated)/spans[index]))
        result.append([round(points[index][k]*(1-t)+points[index+1][k]*t,6) for k in range(3)])
    return result


def road_node(road, points):
    count = road['number_of_lanes']
    fields = {'name': json.dumps(road['name']), 'id': json.dumps(road['id']),
              'width': str(road['width']), 'numberOfLanes': str(count), 'numberOfForwardLanes': str(count//2),
              'lines': lines(count), 'roadBorderHeight': '.035', 'appearance': g.ASPHALT,
              'pavementAppearance': g.PAVEMENT, 'roadBoundingObject': 'TRUE' if road['id'] in ['BLOCK_MID_TUNNEL', 'RECONNECTED_BRIDGE', 'NORTH_WINDING_W', 'NORTH_WINDING_E'] else 'FALSE',
              'rightBorderBoundingObject': 'TRUE', 'leftBorderBoundingObject': 'TRUE',
              'startLine': g.end_markings(road['from'], True, count), 'endLine': g.end_markings(road['to'], False, count)}
    if len(points) == 2 and abs(points[0][2]-points[1][2]) < 1e-8:
        kind = 'StraightRoadSegment'
        fields.update(translation=g.vec(points[0]), rotation=g.vec((0,0,1,math.atan2(points[-1][1]-points[0][1], points[-1][0]-points[0][0]))),
                      length=g.fmt(math.dist(points[0][:2], points[-1][:2])), startingRoadBorderWidth='.14', endingRoadBorderWidth='.14')
    else:
        kind = 'Road'
        fields.update(translation='0 0 0', wayPoints='[\n    '+'\n    '.join(g.vec(p) for p in points)+'\n  ]', splineSubdivision='-1', roadBorderWidth='[ .14 ]')
    return s.Node(kind, kind+' {\n'+'\n'.join('  '+key+' '+value for key,value in fields.items())+'\n}')


def main():
    world = ROOT/'worlds/my_world.wbt'
    data = json.loads((ROOT/'map.json').read_text(encoding='utf-8'))
    if 'expansion' in data:
        print('Expansion already applied; run build_world.py to synchronize edits.')
        return
    backup = Path(tempfile.mkdtemp(prefix='town-before-expansion-'))
    for path in (world, ROOT/'map.json', ROOT/'map.svg'):
        shutil.copy2(path, backup/path.name)
    print('Backup:', backup)
    source = world.read_text(encoding='utf-8')
    native = s.read_nodes(source)
    by_id = {n.get('id'):n for n in native if n.get('id')}
    junctions = data['junctions']
    sign_approaches = {nid:n['ports'][-1]['edge'] for nid,n in junctions.items() if n['ports']}
    old_positions = {nid:n['position'][:] for nid,n in junctions.items()}
    targets = {-27:-35, -17:-23, -8:-12, -1:-1, 3:7, 12:22, 21:35}
    target_y = {8.6:11,15:19,-7.6:-11,-11.6:-16,-13:-17}
    for nid,n in junctions.items():
        if n['kind'] != 'roundabout_port':
            n['position'][0] = targets.get(n['position'][0], n['position'][0])
            n['position'][1] = target_y.get(n['position'][1],n['position'][1])
    for nid,p in {'R_W':[16,0], 'R_N':[22,6], 'R_E':[28,0], 'R_S':[22,-6],
                  'BRIDGE_N':[-23,2.8], 'BRIDGE_S':[-23,-2.8]}.items():
        junctions[nid]['position'] = p
    roads = data['roads']
    full = {}
    for road in roads:
        eid,a,b = road['id'],road['from'],road['to']
        road['number_of_lanes'] = 8 if eid in MIDDLE+['ROUNDABOUT_NORTH','ROUNDABOUT_SOUTH'] else 4
        road['forward_lanes'] = road['number_of_lanes']//2
        road['width'] = road['number_of_lanes']*.6
        pa,pb = junctions[a]['position'],junctions[b]['position']
        xyz = by_id[eid].road_points()
        if eid == 'BLOCK_MID_TUNNEL':
            full[eid] = [[*pa,.008]]+[[p[0]-6,p[1],p[2]] for p in xyz]+[[*pb,.008]]
        elif eid == 'INNER_CURVE':
            full[eid] = [[*p,.008] for p in [pa,(-5,11)]+g.arc((-5,7),4,math.pi/2,0)[1:]+[pb]]
        elif len(xyz) == 2 or eid == 'Y_STEM':
            full[eid] = [[*pa,.008],[*pb,.008]]
        else:
            old = [[*old_positions[a],.008]]+xyz+[[*old_positions[b],.008]]
            distance,total = 0,g.length([p[:2] for p in old])
            moved = []
            for i,p in enumerate(old):
                if i: distance += math.dist(old[i-1][:2],p[:2])
                margin=min(4,total/4)
                t = max(0,min(1,(distance-margin)/(total-2*margin)))
                t = t*t*(3-2*t)
                moved.append([p[k]+(pa[k]-old_positions[a][k])*(1-t)+(pb[k]-old_positions[b][k])*t for k in (0,1)]+[p[2]])
            full[eid] = moved
    # Outer ring and two new rows of connected northern streets.
    for nid,x,y in [('O_W_N',-42,11),('O_W_S',-42,-11),('O_N_W',-16,40),('O_N_M',3,40),('O_N_E',24,40),
                    ('O_E_M',42,0),('O_S_E',22,-24),('O_S_M',-7,-24),
                    ('N1_W',-12,24),('N1_M',7,24),('N1_E',22,24),
                    ('N2_W',-16,32),('N2_M',3,32),('N2_E',24,32)]:
        junctions[nid] = {'position':[x,y], 'kind':'T', 'trim':1.6}
    added = []
    def edge(eid,a,b,points=None,count=4):
        road = {'id':eid,'name':'Road-'+eid,'from':a,'to':b,'number_of_lanes':count,'forward_lanes':count//2,'width':count*.6,'training':False}
        roads.append(road); added.append(eid)
        seq = points or [junctions[a]['position'],junctions[b]['position']]
        full[eid] = [[*p,.008] if len(p)==2 else p for p in seq]
    edge('OUTER_RING_NW','O_W_N','O_N_W',[(-42,11),(-42,36)]+g.arc((-38,36),4,math.pi,math.pi/2)[1:]+[(-16,40)])
    for eid,a,b,amplitude,height in [('NORTH_WINDING_W','O_N_W','O_N_M',1.4,.6),('NORTH_WINDING_E','O_N_M','O_N_E',1.4,.7)]:
        x0,y = junctions[a]['position']; x1,_ = junctions[b]['position']
        seq = []
        for i in range(121):
            t=i/120; x=x0+(x1-x0)*t
            wave=amplitude*math.sin(2*math.pi*t)*math.sin(math.pi*t)**2
            active=max(0,min(1,(x-x0-3)/(x1-x0-6)))
            z=.008+height*math.sin(math.pi*active)**2
            seq.append([x,y+wave,z])
        edge(eid,a,b,seq)
    edge('OUTER_RING_NE','O_N_E','O_E_M',[(24,40),(38,40)]+g.arc((38,36),4,math.pi/2,0)[1:]+[(42,0)])
    edge('OUTER_RING_SE','O_E_M','O_S_E',[(42,0),(42,-20)]+g.arc((38,-20),4,0,-math.pi/2)[1:]+[(22,-24)])
    edge('OUTER_RING_S','O_S_E','O_S_M')
    edge('OUTER_RING_SW','O_S_M','O_W_S',[(-7,-24),(-38,-24)]+g.arc((-38,-20),4,-math.pi/2,-math.pi)[1:]+[(-42,-11)])
    edge('OUTER_RING_W','O_W_S','O_W_N')
    for eid,a,b,count in [('WEST_EXTENSION_N','O_W_N','WEST_UPPER',4),('WEST_EXTENSION_S','O_W_S','WEST_LOWER',4),
                          ('EAST_EXTENSION','EAST','O_E_M',8),('SOUTH_EXTENSION_E','LOWER_EAST','O_S_E',4),('SOUTH_EXTENSION_M','Y_SOUTH','O_S_M',4)]:
        edge(eid,a,b,count=count)
    for suffix in ('W','M','E'):
        edge('NORTH_ACCESS_'+suffix,'NORTH_'+{'W':'WEST','M':'MIDDLE','E':'EAST'}[suffix],'N1_'+suffix)
        a,b='N1_'+suffix,'N2_'+suffix
        pa,pb=junctions[a]['position'],junctions[b]['position']
        edge('NORTH_FAN_'+suffix,a,b)
        edge('NORTH_OUTLET_'+suffix,'N2_'+suffix,'O_N_'+suffix)
    for row in ('N1','N2'):
        edge(row+'_CROSS_W',row+'_W',row+'_M')
        edge(row+'_CROSS_E',row+'_M',row+'_E')
    incident = {nid:[] for nid in junctions}
    for road in roads:
        seq=full[road['id']]
        for nid,p,q in [(road['from'],seq[0],seq[1]),(road['to'],seq[-1],seq[-2])]:
            incident[nid].append((road['id'],g.direction(p[:2],q[:2]),road['width']/2))
    trims = {}
    for nid,ports in incident.items():
        if junctions[nid]['kind'] not in ('roundabout_port','connection'):
            junctions[nid]['kind'] = 'X' if len(ports)==4 else 'T'
        for eid,u,half in ports:
            trim=0 if junctions[nid]['kind'] in ('roundabout_port','connection') else 1.6
            for other,v,other_half in ports:
                sine=abs(u[0]*v[1]-u[1]*v[0])
                if other!=eid and sine>.35 and trim:
                    cosine=u[0]*v[0]+u[1]*v[1]
                    trim=max(trim,((other_half+.14)+(half+.14)*cosine)/sine+.4)
            trims[nid,eid]=trim
    g.junctions.clear();g.junctions.update(junctions)
    road_nodes = {}
    signal_ports = []
    actual_ports = {}
    for road in roads:
        pts=trim3(full[road['id']],trims[road['from'],road['id']],trims[road['to'],road['id']])
        road_nodes[road['id']]=road_node(road,pts)
        for nid,seq in [(road['from'],pts),(road['to'],list(reversed(pts)))]:
            actual_ports[nid,road['id']] = (seq[0][:2],g.direction(seq[0][:2],seq[1][:2]),road['width'])
        if 'SIGNAL_CROSS' in (road['from'],road['to']):
            seq=pts if road['from']=='SIGNAL_CROSS' else list(reversed(pts))
            u=g.direction(seq[0][:2],seq[1][:2])
            signal_ports.append(g.add(seq[0][:2],g.add(g.mul(u,.05),g.mul(g.normal(u),road['width']/2+.22))))
    for node in native:
        eid,name=node.get('id'),node.get('name','')
        if eid in road_nodes:
            node.text=road_nodes.pop(eid).text
        elif eid=='SIGNAL_CROSS':
            node.text='Crossroad {\n  name "SignalCross"\n  id "SIGNAL_CROSS"\n  translation 0 0 .008\n  shape []\n  appearance '+g.ASPHALT+'\n}'
        elif eid=='ROUNDABOUT':
            node.set('translation','22 0 .008');node.set('outerRadius','4.8');node.set('numberOfLanes','4')
            node.set('startRoadsWidth','4.8');node.set('startRoadsNumberOfLanes','8');node.set('startRoadsLine',lines(8));node.set('lines',lines(4));node.set('startRoadsStartLine','[]')
        elif name=='Tunnel-Structure':
            def move(m):
                x,y,z=map(float,m.groups());return 'translation '+g.vec((x-6,y+math.copysign(1.2,y) if abs(y)>1 else y,z))
            node.text=re.sub(r'\btranslation\s+('+s.NUMBER+r')\s+('+s.NUMBER+r')\s+('+s.NUMBER+r')',move,node.text)
            for old,new in [('size 3 2.8 0.05','size 3 5.2 0.05'),('size 0.3 3.14 0.09','size 0.3 5.54 0.09'),('size 0.1 3.2 0.28','size 0.1 5.6 0.28')]:node.text=node.text.replace(old,new)
        elif name.startswith('GROUND-'):
            configs={'GROUND-North':((0,25.325,-.06),(96,45.35,.12)), 'GROUND-South':((0,-16.325,-.06),(96,27.35,.12)),
                     'GROUND-West':((-38.2,0,-.06),(19.6,5.3,.12)), 'GROUND-East':((14.7,0,-.06),(66.6,5.3,.12))}
            position,size=configs[name];node.set('translation',g.vec(position))
            node.text=re.sub(r'\bsize\s+'+s.NUMBER+r'\s+'+s.NUMBER+r'\s+'+s.NUMBER, 'size '+g.vec(size),node.text)
        elif node.type=='Viewpoint':node.set('position','0 9 185')
        elif node.type=='Robot' and 'SIGNAL_SYSTEM' in node.text:
            # Match only the four outer Pose translations, leaving local devices intact.
            positions=iter(signal_ports)
            def lamp(m):
                return '      translation '+g.vec((*next(positions),0))
            node.text,count=re.subn(r'(?m)^      translation\s+'+s.NUMBER+r'\s+'+s.NUMBER+r'\s+'+s.NUMBER,lamp,node.text)
            assert count==4, 'Expected four traffic light outer poses'
        elif node.get('description','').startswith('SIGN_'):
            nid=node.get('description')[5:]
            if nid in old_positions:
                p,u,width=actual_ports[nid,sign_approaches[nid]]
                pos=g.add(p,g.add(g.mul(u,.48),g.mul(g.normal(u),width/2+.19)))
                node.set('translation',g.vec((*pos,0)))
                node.set('rotation',g.vec((0,0,1,math.atan2(u[1],u[0]))))
        elif node.get('description','').startswith(('YIELD_R_','DIRECTION_R_')):
            desc=node.get('description');nid='R_'+desc[-1];p=junctions[nid]['position']
            u=g.direction((22,0),p)
            node.set('translation',g.vec((*g.add(p,g.add(g.mul(u,.25 if desc.startswith('YIELD_') else .55),g.mul(g.normal(u),2.6))),0)))
        elif name=='TRAINING_OBSTACLE':
            p=node.get('translation');p[1]+=4;node.set('translation',g.vec(p))
        elif node.get('description')=='SPEED_LIMIT':
            node.set('translation','-16 17.3 0')
        elif name in ('START_MARKER','GOAL_MARKER'):
            p=node.get('translation');p[0]+=(-8 if name=='START_MARKER' else 14);node.set('translation',g.vec(p))
            if name=='START_MARKER':p[1]=-13.8;node.set('translation',g.vec(p))
    data['bounds']=[-48,48,-30,48]
    data['roundabout'].update(center=[22,0],outer_radius=4.8,ring_lanes=4)
    data['spawn']['position'][0]-=8;data['spawn']['position'][1]=-13.8;data['goal']['position'][0]+=14
    data['expansion']={'prior_bounds':[-30,24,-18,18],'new_road_ids':added,'north_road_ids':['NORTH_WINDING_W','NORTH_WINDING_E'],
                       'middle_road_ids':MIDDLE,'perimeter':added[:8]}
    world.write_text(s.rewrite(source,native)+'\n\n'+'\n\n'.join(n.text for n in road_nodes.values())+'\n',encoding='utf-8')
    (ROOT/'map.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    s.main()


if __name__=='__main__':
    main()
