"""Supervisor world inspector: manages Game-Style 3rd-Person Spring-Arm Chase Camera in normal mode; --verify runs smoke test."""
from controller import Keyboard, Supervisor
from pathlib import Path
import json
import sys
import hashlib
import math


def _yaw_pitch_to_axis_angle(yaw: float, pitch: float) -> list[float]:
    """Convert ENU camera yaw (around Z) and downward pitch (around local Y) to VRML axis-angle."""
    hy = 0.5 * yaw
    hp = 0.5 * pitch
    cy, sy = math.cos(hy), math.sin(hy)
    cp, sp = math.cos(hp), math.sin(hp)
    qw = cy * cp
    qx = -sy * sp
    qy = cy * sp
    qz = sy * cp
    if qw < 0.0:
        qw, qx, qy, qz = -qw, -qx, -qy, -qz
    norm_v = math.sqrt(qx * qx + qy * qy + qz * qz)
    if norm_v < 1e-8:
        return [0.0, 0.0, 1.0, 0.0]
    angle = 2.0 * math.acos(min(1.0, max(-1.0, qw)))
    return [qx / norm_v, qy / norm_v, qz / norm_v, angle]


robot = Supervisor()
dt = int(robot.getBasicTimeStep())
if dt <= 0:
    dt = 16
dt_sec = dt / 1000.0
root = Path(__file__).resolve().parents[2]
if '--verify' not in sys.argv:
    print('MY_WORLD ready: right-hand traffic, T / X / Y junctions, four-arm roundabout.', flush=True)
    kb = Keyboard()
    kb.enable(dt)

    vp_node = robot.getFromDef('OVERVIEW')
    ego_node = robot.getFromDef('EGO_VEHICLE')
    vp_pos_field = vp_node.getField('position') if vp_node is not None else None
    vp_ori_field = vp_node.getField('orientation') if vp_node is not None else None
    ego_custom_field = ego_node.getField('customData') if ego_node is not None else None

    if vp_node is not None:
        follow_field = vp_node.getField('follow')
        if follow_field is not None:
            follow_field.setSFString('')

    PIVOT_FORWARD_M = 1.50
    ARM_DIST_M = 11.0
    ARM_HEIGHT_M = 2.68
    PITCH_RAD = 0.22045
    TAU_TURN_SEC = 0.85
    TAU_STRAIGHT_SEC = 0.50
    TAU_SHIFT_SEC = 0.08
    MAX_YAW_LAG_RAD = 0.52

    cam_yaw = math.pi
    if ego_node is not None:
        r0 = ego_node.getOrientation()
        if r0 and len(r0) >= 4:
            cam_yaw = math.atan2(r0[3], r0[0])

    free_look = False
    free_offset = [ARM_DIST_M, 0.0, ARM_HEIGHT_M]
    free_ori = _yaw_pitch_to_axis_angle(cam_yaw, PITCH_RAD)
    last_cmd_pos = None
    last_cmd_ori = None

    while robot.step(dt) != -1:
        if vp_pos_field is None or vp_ori_field is None or ego_node is None:
            continue

        car_pos = ego_node.getPosition()
        rot = ego_node.getOrientation()
        vel = ego_node.getVelocity()
        if not car_pos or not rot or len(rot) < 9:
            continue

        car_yaw = math.atan2(rot[3], rot[0])
        yaw_rate = vel[5] if (vel and len(vel) >= 6 and math.isfinite(vel[5])) else 0.0

        pivot_x = car_pos[0] + PIVOT_FORWARD_M * rot[0]
        pivot_y = car_pos[1] + PIVOT_FORWARD_M * rot[3]
        pivot_z = car_pos[2] + PIVOT_FORWARD_M * rot[6]

        shift_held = False
        while True:
            k = kb.getKey()
            if k == -1:
                break
            if k == Keyboard.SHIFT or (k & Keyboard.SHIFT) != 0:
                shift_held = True
        if not shift_held and ego_custom_field is not None:
            if ego_custom_field.getSFString() == 'SHIFT:1':
                shift_held = True

        cur_pos = vp_pos_field.getSFVec3f()
        cur_ori = vp_ori_field.getSFRotation()

        if last_cmd_pos is not None and last_cmd_ori is not None and not shift_held:
            pos_diff = math.dist(cur_pos, last_cmd_pos)
            ori_diff = sum(abs(a - b) for a, b in zip(cur_ori, last_cmd_ori))
            if pos_diff > 0.005 or ori_diff > 0.002:
                free_look = True
                free_offset = [cur_pos[0] - pivot_x, cur_pos[1] - pivot_y, cur_pos[2] - pivot_z]
                free_ori = list(cur_ori)
                cam_yaw = math.atan2(-free_offset[1], -free_offset[0])
                last_cmd_pos = list(cur_pos)
                last_cmd_ori = list(cur_ori)
                continue

        if shift_held:
            free_look = False

        if free_look:
            new_pos = [pivot_x + free_offset[0], pivot_y + free_offset[1], pivot_z + free_offset[2]]
            new_ori = list(free_ori)
        else:
            yaw_err = math.atan2(math.sin(car_yaw - cam_yaw), math.cos(car_yaw - cam_yaw))
            if shift_held:
                tau = TAU_SHIFT_SEC
            elif abs(yaw_rate) > 0.08:
                tau = TAU_TURN_SEC
            else:
                tau = TAU_STRAIGHT_SEC

            alpha = 1.0 - math.exp(-dt_sec / tau)
            cam_yaw += alpha * yaw_err

            if not shift_held:
                rem_err = math.atan2(math.sin(car_yaw - cam_yaw), math.cos(car_yaw - cam_yaw))
                if rem_err > MAX_YAW_LAG_RAD:
                    cam_yaw = car_yaw - MAX_YAW_LAG_RAD
                elif rem_err < -MAX_YAW_LAG_RAD:
                    cam_yaw = car_yaw + MAX_YAW_LAG_RAD

            cam_yaw = math.atan2(math.sin(cam_yaw), math.cos(cam_yaw))
            new_pos = [
                pivot_x - ARM_DIST_M * math.cos(cam_yaw),
                pivot_y - ARM_DIST_M * math.sin(cam_yaw),
                pivot_z + ARM_HEIGHT_M,
            ]
            new_ori = _yaw_pitch_to_axis_angle(cam_yaw, PITCH_RAD)

        vp_pos_field.setSFVec3f(new_pos)
        vp_ori_field.setSFRotation(new_ori)
        last_cmd_pos = list(new_pos)
        last_cmd_ori = list(new_ori)
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
            sign=next(children.getMFNode(i) for i in range(children.getCount())
                      if children.getMFNode(i).getField('description') is not None
                      and children.getMFNode(i).getField('description').getSFString()=='SIGN_BLOCK_NORTH')
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
