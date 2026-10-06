"""Merge the two northern rows and split the central four lanes over the crossing."""
import json
import math
import shutil
import tempfile
from pathlib import Path
import build_world as g
import sync_world as s
from expand_world import road_node, resample, trim3


COMPACT_GEOMETRY = {'toe_x': [.25, 13.75], 'plateau_x': [3.25, 10.75],
                    'deck_height_m': .38, 'deck_thickness_m': .06, 'structural_inset_m': .006}


def flyover_points(metadata):
    west, east = metadata['toe_x']
    flat_west, flat_east = metadata['plateau_x']
    count = round((east-west)/.125)
    points = []
    for i in range(count+1):
        x = west+(east-west)*i/count
        t = max(0, min(1, (x-west)/(flat_west-west) if x < flat_west
                       else (east-x)/(east-flat_east) if x > flat_east else 1))
        points.append([round(x, 6), 21.5, round(.008+metadata['deck_height_m']*t*t*(3-2*t), 6)])
    return points


def flyover_structures(bridge, metadata):
    """Return the shallow deck and closed ramp fills below the road collider."""
    west, east = metadata['plateau_x']
    width, y = bridge['width'], bridge['points'][0][1]
    # Keep the structural mesh below asphalt to avoid coplanar depth artefacts.
    profile = [(p[0], z-metadata['structural_inset_m']) for p, z in zip(bridge['points'], bridge['elevations'])]
    top = min(z for x, z in profile if west <= x <= east)
    thickness = metadata['deck_thickness_m']
    name = 'Bridge-'+bridge['id']
    position = ((west+east)/2, y, top-thickness/2)
    size = (east-west, width, thickness)
    result = [f'Solid {{\n  name "{name}-Deck"\n  translation {g.vec(position)}\n'
              f'  children [ Shape {{ appearance {g.PAVEMENT} geometry Box {{ size {g.vec(size)} }} }} ]\n'
              f'  boundingObject Box {{ size {g.vec(size)} }}\n}}']
    for side, section in [('West', [(x, z) for x, z in profile if x <= west]),
                          ('East', [(x, z) for x, z in profile if x >= east])]:
        vertices = [(x, edge, height) for x, z in section
                    for edge, height in [(y-width/2, 0), (y+width/2, 0),
                                         (y-width/2, z), (y+width/2, z)]]
        faces = [[0, 2, 3, 1]]
        for i in range(len(section)-1):
            a, b = 4*i, 4*(i+1)
            faces += [[a, a+1, b+1, b], [a+2, b+2, b+3, a+3],
                      [a, b, b+2, a+2], [a+1, a+3, b+3, b+1]]
        a = 4*(len(section)-1)
        faces.append([a, a+1, a+3, a+2])
        mesh = bridge['id']+'_RAMP_'+side.upper()
        coordinates = '\n      '.join(g.vec(v) for v in vertices)
        indices = '\n      '.join(' '.join(map(str, face))+' -1' for face in faces)
        result.append(f'Solid {{\n  name "{name}-RampSupport-{side}"\n'
                      f'  children [ Shape {{ appearance {g.PAVEMENT} geometry DEF {mesh} IndexedFaceSet {{\n'
                      f'    coord Coordinate {{ point [\n      {coordinates}\n    ] }}\n'
                      f'    coordIndex [\n      {indices}\n    ]\n  }} }} ]\n  boundingObject USE {mesh}\n}}')
    return result


def main():
    root=s.ROOT
    world=root/'worlds/my_world.wbt'
    data=json.loads((root/'map.json').read_text(encoding='utf-8'))
    if 'north_flyover' in data:
        print('Northern flyover already applied; use build_world.py to synchronize.')
        return
    backup=Path(tempfile.mkdtemp(prefix='town-before-flyover-'))
    for p in (world,root/'map.json',root/'map.svg'):
        shutil.copy2(p,backup/p.name)
    print('Backup:',backup)
    source=world.read_text(encoding='utf-8')
    nodes=s.read_nodes(source)
    native={n.get('id'):n for n in nodes if n.get('id')}
    roads={r['id']:r for r in data['roads']}
    junctions=data['junctions']
    removed={'NORTH_TRAINING','NORTH_LINK','N1_CROSS_W','N1_CROSS_E',
             'NORTH_ACCESS_W','NORTH_ACCESS_M','NORTH_ACCESS_E'}
    for key in ('NORTH_WEST','NORTH_MIDDLE','NORTH_EAST'):
        junctions[key]['position'][1]=21.5
    for side,target in [('W','WEST'),('M','MIDDLE'),('E','EAST')]:
        junctions.pop('N1_'+side)
        roads['NORTH_FAN_'+side]['from']='NORTH_'+target
    west, east = COMPACT_GEOMETRY['toe_x']
    for key,x in [('FLYOVER_W',west),('FLYOVER_E',east)]:
        junctions[key]={'position':[x,21.5],'kind':'lane_split','trim':0}
    g.junctions.clear();g.junctions.update(junctions)
    replacements={}
    def save(eid,a,b,points,count=4,forward=None):
        r={'id':eid,'name':'Road-'+eid,'from':a,'to':b,'number_of_lanes':count,
           'forward_lanes':count//2 if forward is None else forward,'width':count*.6}
        node=road_node(r,points)
        node.set('numberOfForwardLanes',str(r['forward_lanes']))
        if count==2:
            node.set('lines','[ RoadLine { width .025 } ]')
            inner='left' if r['forward_lanes']==2 else 'right'
            node.set(inner+'Border','FALSE')
            node.set(inner+'BorderBoundingObject','FALSE')
        for field,nid,incoming in [('startLine',a,count-r['forward_lanes']),('endLine',b,r['forward_lanes'])]:
            if junctions[nid]['kind'] in ('lane_split','connection'):
                node.set(field,'[]')
            elif count==2:
                texture='../../../../../../worlds/textures/stop_line.png'
                node.set(field,json.dumps([texture if incoming else '']*2))
        if eid=='NORTH_FLYOVER':
            node.set('roadBoundingObject','TRUE')
            node.set('roadBorderHeight','.10')
            node.set('roadBorderWidth','[ .10 ]')
        roads[eid]=r; replacements[eid]=node
    # Move existing connections to the merged row, preserving their remote ports.
    for eid in ('OUTER_NW','OUTER_NE','INNER_1','CENTRAL_AXIS','ROUNDABOUT_NORTH',
                'NORTH_FAN_W','NORTH_FAN_M','NORTH_FAN_E'):
        r=roads[eid];seq=native[eid].road_points()
        at_start=r['from'].startswith('NORTH_')
        if at_start:seq=list(reversed(seq))
        nid=r['from'] if at_start else r['to']
        target=junctions[nid]['position']
        if eid in ('OUTER_NW','OUTER_NE'):
            # Smoothly lift only the end of the existing curved approach.
            total=g.length([p[:2] for p in seq])
            old=[p[:] for p in seq];distance=0
            for i,p in enumerate(seq):
                if i:distance+=math.dist(old[i-1][:2],old[i][:2])
                t=max(0,min(1,(distance-(total-10))/10));p[1]+=2.5*t*t*(3-2*t)
            seq=trim3(resample(seq),0,1.8)
        else:
            u=g.direction(target,seq[0][:2])
            trim=4.2 if eid=='NORTH_FAN_M' else 3.3 if nid=='NORTH_MIDDLE' else 3.25
            seq=[seq[0],[target[0]+u[0]*trim,target[1]+u[1]*trim,.008]]
        if at_start:seq=list(reversed(seq))
        save(eid,r['from'],r['to'],seq,r['number_of_lanes'])
    save('NORTH_MERGED_W','NORTH_WEST','FLYOVER_W',[[-10.1,21.5,.008],[west,21.5,.008]],8)
    save('NORTH_MERGED_E','FLYOVER_E','NORTH_EAST',[[east,21.5,.008],[19.0,21.5,.008]],8)
    for suffix,y,forward in [('S',19.7,2),('N',23.3,0)]:
        save('NORTH_GROUND_W_'+suffix,'FLYOVER_W','NORTH_MIDDLE',[[west,y,.008],[3.5,y,.008]],2,forward)
        save('NORTH_GROUND_E_'+suffix,'NORTH_MIDDLE','FLYOVER_E',[[10.5,y,.008],[east,y,.008]],2,forward)
    points=flyover_points(COMPACT_GEOMETRY)
    save('NORTH_FLYOVER','FLYOVER_W','FLYOVER_E',points)
    structures=flyover_structures({'id':'NORTH_FLYOVER','width':2.4,
                                 'points':[p[:2] for p in points],'elevations':[p[2] for p in points]},COMPACT_GEOMETRY)
    for n in nodes:
        eid=n.get('id');desc=n.get('description','')
        if eid in removed or eid in ('J_N1_W','J_N1_M','J_N1_E'):
            n.text=''
        elif eid in replacements:
            n.text=replacements.pop(eid).text
        elif desc in ('SIGN_NORTH_MIDDLE','SIGN_NORTH_EAST'):
            key=desc[5:];r=roads['NORTH_FAN_'+('M' if key.endswith('MIDDLE') else 'E')]
            road=replacements.get(r['id']) or s.read_nodes(next(q.text for q in nodes if q.get('id')==r['id']))[0]
            seq=road.road_points()
            u=g.direction(seq[0][:2],seq[1][:2]);p=g.add(seq[0][:2],g.add(g.mul(u,.48),g.mul(g.normal(u),1.39)))
            n.set('translation',g.vec((*p,0)));n.set('rotation',g.vec((0,0,1,math.atan2(u[1],u[0]))))
        elif n.get('name')=='TRAINING_OBSTACLE':
            n.set('translation','-2.5 17.5 .1')
    for eid in removed:roads.pop(eid)
    added=['NORTH_MERGED_W','NORTH_MERGED_E','NORTH_FLYOVER']+['NORTH_GROUND_'+side+'_'+direction for side in ('W','E') for direction in ('S','N')]
    data['roads']=list(roads.values())
    data['expansion']['new_road_ids']=[eid for eid in data['expansion']['new_road_ids'] if eid not in removed]+added
    data['north_flyover']={'center':[7,21.5],'road':'NORTH_FLYOVER','approaches':['NORTH_MERGED_W','NORTH_MERGED_E'],
                          'ground_roads':added[3:],'splits':['FLYOVER_W','FLYOVER_E'],'ground_junction':'NORTH_MIDDLE',
                          'removed_roads':sorted(removed),**COMPACT_GEOMETRY,
                          'lanes_per_direction':{'bridge':2,'ground':2}}
    world.write_text(s.rewrite(source,nodes)+'\n\n'+'\n\n'.join([n.text for n in replacements.values()]+structures)+'\n',encoding='utf-8')
    (root/'map.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    s.main()


if __name__=='__main__':
    main()
