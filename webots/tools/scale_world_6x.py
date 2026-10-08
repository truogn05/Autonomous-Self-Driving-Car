"""Scale entire Webots simulation town by 6.0x (from 1:10 small scale to 1:1 real scale).

Complies strictly with AGENT.md, GEMINI.md, and PLAN-20261-SCALE-6X-MAP-VERIFIED.
Reads from pre-scale backups (*.bak_pre_scale6) and writes to:
1. webots/worlds/my_world.wbt
2. webots/map.json
3. webots/map-north.json
"""
from pathlib import Path
import json
import math
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
WBT_PATH = ROOT / "worlds" / "my_world.wbt"
MAP_S_PATH = ROOT / "map.json"
MAP_N_PATH = ROOT / "map-north.json"

WBT_BAK = ROOT / "worlds" / "my_world.wbt.bak_pre_scale6"
MAP_S_BAK = ROOT / "map.json.bak_pre_scale6"
MAP_N_BAK = ROOT / "map-north.json.bak_pre_scale6"

SCALE = 6.0

def fmt(x: float) -> str:
    if abs(x) < 1e-12:
        return '0'
    res = f"{x:.6f}".rstrip('0').rstrip('.')
    return res if res else '0'

def scale_numbers(s: str) -> str:
    parts = re.split(r'([-+0-9.eE]+)', s)
    res = []
    for part in parts:
        if re.fullmatch(r'[-+0-9.eE]+', part):
            try:
                v = float(part) * SCALE
                res.append(fmt(v))
            except ValueError:
                res.append(part)
        else:
            res.append(part)
    return ''.join(res)

SCALABLE_BRACKET_FIELDS = {'wayPoints', 'shape', 'path', 'point', 'startRoadsLength'}

SCALABLE_SCALARS = {
    'height', 'radius', 'width', 'length', 'roadBorderHeight',
    'borderWidth', 'borderHeight', 'innerRadius', 'outerRadius',
    'poleGap', 'startRoadsWidth', 'startingRoadBorderWidth', 'endingRoadBorderWidth'
}

re_translation = re.compile(r'^(\s*translation\s+)([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)(.*)$')
re_position = re.compile(r'^(\s*position\s+)([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)(.*)$')
re_location = re.compile(r'^(\s*location\s+)([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)(.*)$')
re_size = re.compile(r'^(\s*size\s+)([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)(.*)$')
re_scalar = re.compile(r'^(\s*)(' + '|'.join(SCALABLE_SCALARS) + r')(\s+)([-+0-9.eE]+)(.*)$')
re_scalable_bracket = re.compile(r'\b(wayPoints|shape|path|point|startRoadsLength)\s*\[(.*)$')

def transform_wbt_text(text: str) -> str:
    lines = text.splitlines()
    output_lines = []
    
    in_bracket_field = None
    bracket_content = []
    bracket_depth = 0
    scale_current_bracket = False
    
    recent_node_context = []
    
    for line in lines:
        stripped = line.strip()
        
        m_node = re.search(r'\b(Coordinate|TextureCoordinate|IndexedFaceSet|Road|Crossroad|Roundabout|CrashBarrier)\b', line)
        if m_node:
            recent_node_context.append(m_node.group(1))
            if len(recent_node_context) > 10:
                recent_node_context.pop(0)

        if in_bracket_field:
            bracket_depth += line.count('[') - line.count(']')
            bracket_content.append(line)
            if bracket_depth <= 0:
                full_block = '\n'.join(bracket_content)
                if scale_current_bracket:
                    scaled_block = scale_numbers(full_block)
                    output_lines.append(scaled_block)
                else:
                    output_lines.append(full_block)
                in_bracket_field = None
                bracket_content = []
                scale_current_bracket = False
            continue
            
        m_bracket = re_scalable_bracket.search(line)
        if m_bracket:
            field_name = m_bracket.group(1)
            rest = m_bracket.group(2)
            
            should_scale = True
            if field_name == 'point':
                is_texture = any(c == 'TextureCoordinate' for c in recent_node_context[-3:]) or 'texCoord' in line or 'TextureCoordinate' in line
                if is_texture:
                    should_scale = False
            
            b_depth = 1 + rest.count('[') - rest.count(']')
            if b_depth <= 0:
                if should_scale:
                    output_lines.append(scale_numbers(line))
                else:
                    output_lines.append(line)
                continue
            else:
                in_bracket_field = field_name
                bracket_depth = b_depth
                scale_current_bracket = should_scale
                bracket_content = [line]
                continue

        m = re_translation.match(line)
        if m:
            x, y, z = float(m.group(2)) * SCALE, float(m.group(3)) * SCALE, float(m.group(4)) * SCALE
            output_lines.append(f"{m.group(1)}{fmt(x)} {fmt(y)} {fmt(z)}{m.group(5)}")
            continue
            
        m = re_position.match(line)
        if m:
            x, y, z = float(m.group(2)) * SCALE, float(m.group(3)) * SCALE, float(m.group(4)) * SCALE
            output_lines.append(f"{m.group(1)}{fmt(x)} {fmt(y)} {fmt(z)}{m.group(5)}")
            continue

        m = re_location.match(line)
        if m:
            x, y, z = float(m.group(2)) * SCALE, float(m.group(3)) * SCALE, float(m.group(4)) * SCALE
            output_lines.append(f"{m.group(1)}{fmt(x)} {fmt(y)} {fmt(z)}{m.group(5)}")
            continue

        m = re_size.match(line)
        if m:
            x, y, z = float(m.group(2)) * SCALE, float(m.group(3)) * SCALE, float(m.group(4)) * SCALE
            output_lines.append(f"{m.group(1)}{fmt(x)} {fmt(y)} {fmt(z)}{m.group(5)}")
            continue

        m = re_scalar.match(line)
        if m:
            v = float(m.group(4)) * SCALE
            output_lines.append(f"{m.group(1)}{m.group(2)}{m.group(3)}{fmt(v)}{m.group(5)}")
            continue

        output_lines.append(line)

    return '\n'.join(output_lines)

def scale_point2d(pt):
    return [round(pt[0] * SCALE, 6), round(pt[1] * SCALE, 6)]

def scale_point3d(pt):
    return [round(pt[0] * SCALE, 6), round(pt[1] * SCALE, 6), round(pt[2] * SCALE, 6)]

def scale_list_of_points2d(pts):
    return [scale_point2d(p) for p in pts]

def scale_elevations(elevs):
    return [round(e * SCALE, 6) for e in elevs]

def road_height(point, roads):
    best, height = math.inf, 0.048
    for road in roads:
        for i, (a, b) in enumerate(zip(road['points'], road['points'][1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            if dx*dx + dy*dy == 0:
                continue
            t = max(0, min(1, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
            distance = (point[0]-a[0]-t*dx)**2+(point[1]-a[1]-t*dy)**2
            if distance < best:
                best = distance
                height = road['elevations'][i]*(1-t)+road['elevations'][i+1]*t
    return height

def transform_map_json(data):
    d = json.loads(json.dumps(data))
    
    if "bounds" in d and len(d["bounds"]) == 4:
        d["bounds"] = [round(x * SCALE, 6) for x in d["bounds"]]
        
    if "road_scale" in d:
        d["road_scale"] = d["road_scale"] * SCALE
        
    if "roundabout" in d:
        rb = d["roundabout"]
        if "center" in rb:
            rb["center"] = scale_point2d(rb["center"])
        if "inner_radius" in rb:
            rb["inner_radius"] = round(rb["inner_radius"] * SCALE, 6)
        if "outer_radius" in rb:
            rb["outer_radius"] = round(rb["outer_radius"] * SCALE, 6)
        if "ports" in rb:
            for p in rb["ports"]:
                if "position" in p:
                    p["position"] = scale_point2d(p["position"])
                if "width" in p:
                    p["width"] = round(p["width"] * SCALE, 6)

    if "spawn" in d:
        sp = d["spawn"]
        if "point" in sp:
            sp["point"] = scale_point2d(sp["point"])

    if "goal" in d:
        g = d["goal"]
        if "point" in g:
            g["point"] = scale_point2d(g["point"])

    if "junctions" in d:
        for j_id, j in d["junctions"].items():
            if "position" in j:
                j["position"] = scale_point2d(j["position"])
            if "shape" in j:
                j["shape"] = scale_list_of_points2d(j["shape"])
            if "polygon" in j:
                j["polygon"] = scale_list_of_points2d(j["polygon"])
            if "trim" in j:
                j["trim"] = round(j["trim"] * SCALE, 6)
            if "ports" in j:
                for p in j["ports"]:
                    if "position" in p:
                        p["position"] = scale_point2d(p["position"])
                    if "width" in p:
                        p["width"] = round(p["width"] * SCALE, 6)

    if "training" in d and "lane_width" in d["training"]:
        d["training"]["lane_width"] = round(d["training"]["lane_width"] * SCALE, 6)

    if "roads" in d:
        for r in d["roads"]:
            if "width" in r:
                r["width"] = round(r["width"] * SCALE, 6)
            if "points" in r:
                r["points"] = scale_list_of_points2d(r["points"])
            if "elevations" in r:
                r["elevations"] = scale_elevations(r["elevations"])

    if "lanes" in d and "roads" in d:
        roads_map = {e['id']: e for e in d['roads']}
        for l_id, l in d["lanes"].items():
            if "points" in l:
                l["points"] = scale_list_of_points2d(l["points"])
            support = [roads_map[l['road_id']]] if l.get('road_id') in roads_map else []
            l["elevations"] = [round(road_height(p, support) if support else 0.048, 6) for p in l["points"]]

    if "north_flyover" in d:
        nf = d["north_flyover"]
        if "center" in nf:
            nf["center"] = scale_point2d(nf["center"])
        if "toe_x" in nf:
            nf["toe_x"] = [round(x * SCALE, 6) for x in nf["toe_x"]]
        if "plateau_x" in nf:
            nf["plateau_x"] = [round(x * SCALE, 6) for x in nf["plateau_x"]]
        for k in ["deck_height_m", "deck_thickness_m", "structural_inset_m"]:
            if k in nf:
                nf[k] = round(nf[k] * SCALE, 6)

    if "expansion" in d:
        exp = d["expansion"]
        if "prior_bounds" in exp:
            exp["prior_bounds"] = [round(x * SCALE, 6) for x in exp["prior_bounds"]]

    if "medians" in d:
        for m in d["medians"]:
            if "width" in m:
                m["width"] = round(m["width"] * SCALE, 6)
            if "height" in m:
                m["height"] = round(m["height"] * SCALE, 6)
            if "sections" in m:
                m["sections"] = [[[round(coord * SCALE, 6) for coord in pt] for pt in sec] for sec in m["sections"]]

    return d

def transform_map_north_json(data):
    d = json.loads(json.dumps(data))
    
    if "bounds" in d and len(d["bounds"]) == 4:
        d["bounds"] = [round(x * SCALE, 6) for x in d["bounds"]]

    if "river" in d:
        rv = d["river"]
        if "water_elevation" in rv:
            rv["water_elevation"] = round(rv["water_elevation"] * SCALE, 6)
        if "bed_elevation" in rv:
            rv["bed_elevation"] = round(rv["bed_elevation"] * SCALE, 6)
        rv["s_formula"] = "309.0 + 12.0 * sin(2*pi*(x+312)/624)"
        rv["w_formula"] = "78.0 + 12.0 * sin(4*pi*(x+312)/624 + pi/3)"

    if "flyover" in d:
        fo = d["flyover"]
        if "span_y" in fo:
            fo["span_y"] = [round(y * SCALE, 6) for y in fo["span_y"]]
        if "plateau_y" in fo:
            fo["plateau_y"] = [round(y * SCALE, 6) for y in fo["plateau_y"]]
        fo["top_elevation"] = 3.12
        if "deck_clearance" in fo:
            fo["deck_clearance"] = round(fo["deck_clearance"] * SCALE, 6)

    if "underpass" in d:
        up = d["underpass"]
        if "span_y" in up:
            up["span_y"] = [round(y * SCALE, 6) for y in up["span_y"]]
        if "floor_y" in up:
            up["floor_y"] = [round(y * SCALE, 6) for y in up["floor_y"]]
        up["floor_elevation"] = -3.60
        if "roof_clearance" in up:
            up["roof_clearance"] = round(up["roof_clearance"] * SCALE, 6)

    if "junctions" in d:
        for j_id, j in d["junctions"].items():
            if "position" in j:
                j["position"] = scale_point2d(j["position"])
            if "shape" in j:
                j["shape"] = scale_list_of_points2d(j["shape"])
            if "ports" in j:
                for p in j["ports"]:
                    if "position" in p:
                        p["position"] = scale_point2d(p["position"])
                    if "width" in p:
                        p["width"] = round(p["width"] * SCALE, 6)

    if "roads" in d:
        for r in d["roads"]:
            if "width" in r:
                r["width"] = round(r["width"] * SCALE, 6)
            if "points" in r:
                r["points"] = scale_list_of_points2d(r["points"])
            if "elevations" in r:
                r["elevations"] = scale_elevations(r["elevations"])

    if "lanes" in d:
        for l_id, l in d["lanes"].items():
            if "points" in l:
                l["points"] = scale_list_of_points2d(l["points"])
            if "elevations" in l:
                l["elevations"] = scale_elevations(l["elevations"])

    return d

def main():
    print(f"=== Scaling world by factor {SCALE} (from backups) ===")
    
    # 1. Transform WBT from backup
    print("Reading my_world.wbt.bak_pre_scale6...")
    wbt_text = WBT_BAK.read_text(encoding="utf-8")
    scaled_wbt = transform_wbt_text(wbt_text)
    WBT_PATH.write_text(scaled_wbt, encoding="utf-8")
    print(f"Updated {WBT_PATH} successfully ({len(scaled_wbt.splitlines())} lines).")

    # 2. Transform map.json from backup
    print("Updating map.json from backup...")
    with open(MAP_S_BAK, "r", encoding="utf-8") as f:
        map_s = json.load(f)
    map_s_scaled = transform_map_json(map_s)
    with open(MAP_S_PATH, "w", encoding="utf-8") as f:
        json.dump(map_s_scaled, f, indent=2)
    print(f"Updated {MAP_S_PATH} successfully.")

    # 3. Transform map-north.json from backup
    print("Updating map-north.json from backup...")
    with open(MAP_N_BAK, "r", encoding="utf-8") as f:
        map_n = json.load(f)
    map_n_scaled = transform_map_north_json(map_n)
    with open(MAP_N_PATH, "w", encoding="utf-8") as f:
        json.dump(map_n_scaled, f, indent=2)
    print(f"Updated {MAP_N_PATH} successfully.")

    print("\nScale x6 execution completed successfully!")

if __name__ == "__main__":
    main()
