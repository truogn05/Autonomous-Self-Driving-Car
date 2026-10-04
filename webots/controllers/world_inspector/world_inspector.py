"""Passive in the user world; --verify runs a finite simulator smoke test."""
from controller import Supervisor
from pathlib import Path
import json
import sys
import hashlib
import math

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
root = Path(__file__).resolve().parents[2]
if '--verify' not in sys.argv:
    print('MY_WORLD ready: right-hand traffic, T / X / Y junctions, four-arm roundabout.', flush=True)
    while robot.step(dt) != -1:
        pass
else:
    results = {'errors': [], 'phases': [], 'probes': [],
               'world_sha256':hashlib.sha256((root/'worlds/my_world.wbt').read_bytes()).hexdigest()}
    children = robot.getRoot().getField('children')
    map_data=json.loads((root/'map.json').read_text(encoding='utf-8'))
    scale=map_data.get('road_scale',1)
    probes=[(-10.2*scale,-5.3*scale,.04),(-7*scale,0,.04),(-4*scale,0,.04),
            (-3.5*scale,-5.8*scale,.04),(6*scale,1.6*scale,.04),(1.5*scale,4*scale,.04),
            (-1.25*scale,7.35*scale,.04),(-2.5,14.1,.24),
            (-10.5*scale-.6*scale-.07,-5.3*scale,.083),(6*scale+1.2*scale-.05,0,.088)]
    for i,(x,y,expected) in enumerate(probes):
        children.importMFNodeFromString(-1, f'''DEF PROBE_{i} Solid {{ translation {x} {y} .38
            children [ Shape {{ appearance PBRAppearance {{ baseColor 1 .2 .1 }} geometry Sphere {{ radius .04 }} }} ]
            boundingObject Sphere {{ radius .04 }} physics Physics {{ density -1 mass .05 }} }}''')
    observed=set()
    while robot.step(dt) != -1:
        if robot.getTime()>1 and not results['probes']:
            for i,(_,_,expected) in enumerate(probes):
                node=robot.getFromDef(f'PROBE_{i}')
                z=node.getPosition()[2]
                results['probes'].append({'id':i,'z':z,'contacts':len(node.getContactPoints())})
                if abs(z-expected)>.015:
                    results['errors'].append(f'Collision probe {i} at z={z}, expected {expected}')
        signal=robot.getFromDef('SIGNAL_SYSTEM')
        raw=signal.getField('customData').getSFString()
        if raw:
            state=json.loads(raw)
            phase=(state['EW'],state['NS'])
            if phase not in observed:
                observed.add(phase); results['phases'].append(state)
            if state['EW']!='red' and state['NS']!='red':
                results['errors'].append('Conflicting signal phases')
        if robot.getTime()>27:
            if observed != {('green','red'),('amber','red'),('red','red'),('red','green'),('red','amber')}:
                results['errors'].append('Missing signal phases')
            (root/'validation').mkdir(exist_ok=True)
            for i in range(len(probes)):
                robot.getFromDef(f'PROBE_{i}').remove()
            robot.getSelf().getField('children').importMFNodeFromString(-1, '''DEF SURVEY_CAMERA Camera {
              translation 0 0 68 rotation -.577350269 .577350269 .577350269 2.094395102
              name "survey" width 1600 height 1200 fieldOfView .8 near .05 }''')
            robot.step(dt)
            camera=robot.getDevice('survey')
            camera.enable(dt)
            for _ in range(3):
                robot.step(dt)
            camera.saveImage(str(root/'validation'/'town-overview.png'),100)
            camera_node=robot.getFromDef('SURVEY_CAMERA')
            sign=robot.getFromDef('SIGN_BLOCK_NORTH')
            sx,sy,_=sign.getPosition()
            theta=sign.getField('rotation').getSFRotation()[3]
            camera_node.getField('translation').setSFVec3f([sx+1.2*math.cos(theta),sy+1.2*math.sin(theta),.15])
            camera_node.getField('rotation').setSFRotation([0,0,1,theta+math.pi])
            camera_node.getField('fieldOfView').setSFFloat(1.2)
            for _ in range(3):
                robot.step(dt)
            camera.saveImage(str(root/'validation'/'robot-eye-sign.png'),100)
            (root/'validation'/'runtime.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
            robot.exportImage(str(root/'validation'/'webots-overview.png'),100)
            print('WORLD_CHECK '+json.dumps(results),flush=True)
            robot.simulationQuit(1 if results['errors'] else 0)
            break
