"""Upgrade both flyovers in South and North districts to doubled height (Z=6.24m) and smooth circular/cosine arch.

Complies strictly with AGENT.md, GEMINI.md, and PLAN-20261-FLYOVERS-ARCH-2X-HEIGHT.
"""
import copy
import json
import math
import re
import shutil
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WBT_PATH = ROOT / "worlds" / "my_world.wbt"
MAP_S_PATH = ROOT / "map.json"
MAP_N_PATH = ROOT / "map-north.json"
CHK_GEO_PATH = ROOT / "tools" / "check_geometry.py"
CHK_ND_PATH = ROOT / "tools" / "check_north_district.py"

# Backups
for p in [WBT_PATH, MAP_S_PATH, MAP_N_PATH, CHK_GEO_PATH, CHK_ND_PATH]:
    bak = p.with_suffix(p.suffix + ".bak_pre_arch2x")
    if not bak.exists():
        shutil.copy2(p, bak)
        print(f"Created backup: {bak.name}")

L = 81.0
R = 40.5
z_road = 0.048
z_peak = 6.240
h = z_peak - z_road # 6.192
inset = 0.036
thickness = 0.36
half_w = 7.2

def get_z(u):
    if abs(u) >= R:
        return z_road
    return z_road + h * 0.5 * (1.0 + math.cos(math.pi * u / R))

num_pts = 109
points_u = [-R + (2 * R) * i / (num_pts - 1) for i in range(num_pts)]
elevs = [round(get_z(u), 6) for u in points_u]

def fmt(val):
    if abs(val) < 1e-12:
        return '0'
    res = f"{val:.6f}".rstrip('0').rstrip('.')
    return res if res else '0'

def format_ifs(def_name, vertices, faces):
    coord_lines = "      " + "\n      ".join(f"{fmt(v[0])} {fmt(v[1])} {fmt(v[2])}" for v in vertices)
    face_lines = "      " + "\n      ".join(" ".join(map(str, f)) + " -1" for f in faces)
    return f"""geometry DEF {def_name} IndexedFaceSet {{
    coord Coordinate {{
      point [
{coord_lines}
      ]
    }}
    coordIndex [
{face_lines}
    ]
  }}"""

def replace_road_waypoints(text, road_name, new_wps):
    idx = text.find(f'name "{road_name}"')
    assert idx != -1, f'{road_name} not found'
    wp_idx = text.find('wayPoints [', idx)
    assert wp_idx != -1, f'wayPoints [ not found for {road_name}'
    bracket_depth = 0
    end_idx = wp_idx
    for i in range(wp_idx + len('wayPoints'), len(text)):
        if text[i] == '[':
            bracket_depth += 1
        elif text[i] == ']':
            bracket_depth -= 1
            if bracket_depth == 0:
                end_idx = i + 1
                break
    wps_text = "wayPoints [\n    " + ",\n    ".join(new_wps) + "\n  ]"
    return text[:wp_idx] + wps_text + text[end_idx:]

def replace_solid(text, name, solid_text):
    idx = text.find(f'name "{name}"')
    assert idx != -1, f'{name} not found'
    start = text.rfind('Solid {', 0, idx)
    assert start != -1, f'Solid {{ start not found for {name}'
    bracket_depth = 0
    end = start
    for i in range(start, len(text)):
        if text[i] == '{':
            bracket_depth += 1
        elif text[i] == '}':
            bracket_depth -= 1
            if bracket_depth == 0:
                end = i + 1
                break
    return text[:start] + solid_text + text[end:]

def main():
    print("=== Upgrading Flyovers to Doubled Height (Z=6.24m) & Cosine Arch ===")
    wbt_text = WBT_PATH.read_text(encoding="utf-8")
    
    # 1. Update waypoints
    wps_s = [f"{fmt(42.0 + points_u[i])} 129 {fmt(elevs[i])}" for i in range(109)]
    wps_n = [f"105 {fmt(669.0 + points_u[i])} {fmt(elevs[i])}" for i in range(109)]
    wbt_text = replace_road_waypoints(wbt_text, "Road-NORTH_FLYOVER", wps_s)
    wbt_text = replace_road_waypoints(wbt_text, "Road-ND_FLYOVER", wps_n)
    print("Updated road waypoints in WBT.")

    # 2. South meshes
    y_center_s = 129.0
    x_pts_s = [42.0 + u for u in points_u]

    # Median South
    v_med_s, f_med_s = [], []
    for i in range(109):
        x = x_pts_s[i]
        z = elevs[i]
        v_med_s.extend([
            [x, y_center_s - 0.3, z],
            [x, y_center_s + 0.3, z],
            [x, y_center_s + 0.3, z + 0.6],
            [x, y_center_s - 0.3, z + 0.6]
        ])
    f_med_s.append([0, 3, 2, 1])
    for i in range(108):
        a = 4 * i
        b = 4 * (i + 1)
        f_med_s.append([a, a+1, b+1, b])
        f_med_s.append([a+2, a+3, b+3, b+2])
        f_med_s.append([a, b, b+3, a+3])
        f_med_s.append([a+1, a+2, b+2, b+1])
    a = 4 * 108
    f_med_s.append([a, a+1, a+2, a+3])

    solid_med_s = f"""Solid {{
  name "CentreMedian-NORTH_FLYOVER"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('MEDIAN_NORTH_FLYOVER', v_med_s, f_med_s)}
    }}
  ]
  boundingObject USE MEDIAN_NORTH_FLYOVER
}}"""
    wbt_text = replace_solid(wbt_text, "CentreMedian-NORTH_FLYOVER", solid_med_s)

    # Ramp West
    v_rw_s, f_rw_s = [], []
    for i in range(25):
        x = x_pts_s[i]
        z_top = elevs[i] - inset
        v_rw_s.extend([
            [x, y_center_s - half_w, 0.0],
            [x, y_center_s + half_w, 0.0],
            [x, y_center_s - half_w, z_top],
            [x, y_center_s + half_w, z_top]
        ])
    f_rw_s.append([0, 2, 3, 1])
    for i in range(24):
        a = 4 * i
        b = 4 * (i + 1)
        f_rw_s.append([a, a+1, b+1, b])
        f_rw_s.append([a+2, b+2, b+3, a+3])
        f_rw_s.append([a, b, b+2, a+2])
        f_rw_s.append([a+1, a+3, b+3, b+1])
    a = 4 * 24
    f_rw_s.append([a, a+1, a+3, a+2])

    solid_rw_s = f"""Solid {{
  name "Bridge-NORTH_FLYOVER-RampSupport-West"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('NORTH_FLYOVER_RAMP_WEST', v_rw_s, f_rw_s)}
    }}
  ]
  boundingObject USE NORTH_FLYOVER_RAMP_WEST
}}"""
    wbt_text = replace_solid(wbt_text, "Bridge-NORTH_FLYOVER-RampSupport-West", solid_rw_s)

    # Ramp East
    v_re_s, f_re_s = [], []
    for idx_i, i in enumerate(range(84, 109)):
        x = x_pts_s[i]
        z_top = elevs[i] - inset
        v_re_s.extend([
            [x, y_center_s - half_w, 0.0],
            [x, y_center_s + half_w, 0.0],
            [x, y_center_s - half_w, z_top],
            [x, y_center_s + half_w, z_top]
        ])
    f_re_s.append([0, 2, 3, 1])
    for i in range(24):
        a = 4 * i
        b = 4 * (i + 1)
        f_re_s.append([a, a+1, b+1, b])
        f_re_s.append([a+2, b+2, b+3, a+3])
        f_re_s.append([a, b, b+2, a+2])
        f_re_s.append([a+1, a+3, b+3, b+1])
    a = 4 * 24
    f_re_s.append([a, a+1, a+3, a+2])

    solid_re_s = f"""Solid {{
  name "Bridge-NORTH_FLYOVER-RampSupport-East"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('NORTH_FLYOVER_RAMP_EAST', v_re_s, f_re_s)}
    }}
  ]
  boundingObject USE NORTH_FLYOVER_RAMP_EAST
}}"""
    wbt_text = replace_solid(wbt_text, "Bridge-NORTH_FLYOVER-RampSupport-East", solid_re_s)

    # Deck South
    v_dk_s, f_dk_s = [], []
    for idx_i, i in enumerate(range(24, 85)):
        x = x_pts_s[i]
        z_top = elevs[i] - inset
        z_bot = z_top - thickness
        v_dk_s.extend([
            [x, y_center_s - half_w, z_bot],
            [x, y_center_s + half_w, z_bot],
            [x, y_center_s - half_w, z_top],
            [x, y_center_s + half_w, z_top]
        ])
    f_dk_s.append([0, 2, 3, 1])
    for i in range(60):
        a = 4 * i
        b = 4 * (i + 1)
        f_dk_s.append([a, a+1, b+1, b])
        f_dk_s.append([a+2, b+2, b+3, a+3])
        f_dk_s.append([a, b, b+2, a+2])
        f_dk_s.append([a+1, a+3, b+3, b+1])
    a = 4 * 60
    f_dk_s.append([a, a+1, a+3, a+2])

    solid_dk_s = f"""Solid {{
  name "Bridge-NORTH_FLYOVER-Deck"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('NORTH_FLYOVER_DECK', v_dk_s, f_dk_s)}
    }}
  ]
  boundingObject USE NORTH_FLYOVER_DECK
}}"""
    wbt_text = replace_solid(wbt_text, "Bridge-NORTH_FLYOVER-Deck", solid_dk_s)

    # 3. North meshes
    x_center_n = 105.0
    y_pts_n = [669.0 + u for u in points_u]

    # Ramp South
    v_rs_s, f_rs_s = [], []
    for i in range(25):
        y = y_pts_n[i]
        z_top = elevs[i] - inset
        v_rs_s.extend([
            [x_center_n - half_w, y, z_top],
            [x_center_n - half_w, y, 0.0],
            [x_center_n + half_w, y, z_top],
            [x_center_n + half_w, y, 0.0]
        ])
    f_rs_s.append([0, 1, 3, 2])
    for i in range(24):
        a = 4 * i
        b = 4 * (i + 1)
        f_rs_s.append([a, a+2, b+2, b])
        f_rs_s.append([a+1, b+1, b+3, a+3])
        f_rs_s.append([a, b, b+1, a+1])
        f_rs_s.append([a+2, a+3, b+3, b+2])
    a = 4 * 24
    f_rs_s.append([a, a+2, a+3, a+1])

    solid_rs_s = f"""Solid {{
  name "ND_Bridge-RampSupport-South"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('ND_RAMP_SUPPORT_SOUTH', v_rs_s, f_rs_s)}
    }}
  ]
  boundingObject USE ND_RAMP_SUPPORT_SOUTH
}}"""
    wbt_text = replace_solid(wbt_text, "ND_Bridge-RampSupport-South", solid_rs_s)

    # Ramp North
    v_rs_n, f_rs_n = [], []
    for idx_i, i in enumerate(range(84, 109)):
        y = y_pts_n[i]
        z_top = elevs[i] - inset
        v_rs_n.extend([
            [x_center_n - half_w, y, z_top],
            [x_center_n - half_w, y, 0.0],
            [x_center_n + half_w, y, z_top],
            [x_center_n + half_w, y, 0.0]
        ])
    f_rs_n.append([0, 1, 3, 2])
    for i in range(24):
        a = 4 * i
        b = 4 * (i + 1)
        f_rs_n.append([a, a+2, b+2, b])
        f_rs_n.append([a+1, b+1, b+3, a+3])
        f_rs_n.append([a, b, b+1, a+1])
        f_rs_n.append([a+2, a+3, b+3, b+2])
    a = 4 * 24
    f_rs_n.append([a, a+2, a+3, a+1])

    solid_rs_n = f"""Solid {{
  name "ND_Bridge-RampSupport-North"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('ND_RAMP_SUPPORT_NORTH', v_rs_n, f_rs_n)}
    }}
  ]
  boundingObject USE ND_RAMP_SUPPORT_NORTH
}}"""
    wbt_text = replace_solid(wbt_text, "ND_Bridge-RampSupport-North", solid_rs_n)

    # Deck North
    v_dk_n, f_dk_n = [], []
    for idx_i, i in enumerate(range(24, 85)):
        y = y_pts_n[i]
        z_top = elevs[i] - inset
        z_bot = z_top - thickness
        v_dk_n.extend([
            [x_center_n - half_w, y, z_top],
            [x_center_n - half_w, y, z_bot],
            [x_center_n + half_w, y, z_top],
            [x_center_n + half_w, y, z_bot]
        ])
    f_dk_n.append([0, 1, 3, 2])
    for i in range(60):
        a = 4 * i
        b = 4 * (i + 1)
        f_dk_n.append([a, a+2, b+2, b])
        f_dk_n.append([a+1, b+1, b+3, a+3])
        f_dk_n.append([a, b, b+1, a+1])
        f_dk_n.append([a+2, a+3, b+3, b+2])
    a = 4 * 60
    f_dk_n.append([a, a+2, a+3, a+1])

    solid_dk_n = f"""Solid {{
  name "ND_Bridge-Deck"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('ND_DECK', v_dk_n, f_dk_n)}
    }}
  ]
  boundingObject USE ND_DECK
}}"""
    wbt_text = replace_solid(wbt_text, "ND_Bridge-Deck", solid_dk_n)

    # Median North
    v_med_n, f_med_n = [], []
    for i in range(109):
        y = y_pts_n[i]
        zb = elevs[i]
        zt = zb + 0.60
        v_med_n.extend([
            [104.7, y, zt],
            [104.7, y, zb],
            [105.3, y, zt],
            [105.3, y, zb]
        ])
    for j in range(108):
        wt_0, wb_0, et_0, eb_0 = 4*j, 4*j+1, 4*j+2, 4*j+3
        wt_1, wb_1, et_1, eb_1 = 4*(j+1), 4*(j+1)+1, 4*(j+1)+2, 4*(j+1)+3
        f_med_n.append([wt_0, wt_1, et_1, et_0])
        f_med_n.append([wb_0, eb_0, eb_1, wb_1])
        f_med_n.append([wb_0, wb_1, wt_1, wt_0])
        f_med_n.append([et_0, et_1, eb_1, eb_0])
    f_med_n.append([1, 0, 2, 3])
    K = 4 * 108
    f_med_n.append([K+0, K+1, K+3, K+2])

    solid_med_n = f"""Solid {{
  name "ND_Median-FLYOVER"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      {format_ifs('ND_MESH_MEDIAN_FLYOVER', v_med_n, f_med_n)}
    }}
  ]
  boundingObject USE ND_MESH_MEDIAN_FLYOVER
}}"""
    wbt_text = replace_solid(wbt_text, "ND_Median-FLYOVER", solid_med_n)

    # Write updated WBT
    WBT_PATH.write_text(wbt_text, encoding="utf-8")
    print(f"Updated {WBT_PATH} successfully.")

    # 4. Update map.json
    map_s = json.loads(MAP_S_PATH.read_text(encoding="utf-8"))
    r_s = next(r for r in map_s['roads'] if r['id'] == 'NORTH_FLYOVER')
    r_s['elevations'] = elevs
    map_s['north_flyover']['deck_height_m'] = 6.192
    map_s['north_flyover']['top_elevation'] = 6.240
    map_s['north_flyover']['deck_clearance'] = 5.381
    map_s['north_flyover']['max_slope'] = 0.24

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

    for l in map_s.get('lanes', {}).values():
        if l.get('road_id') == 'NORTH_FLYOVER':
            l['elevations'] = [round(road_height(p, [r_s]), 6) for p in l['points']]

    for m in map_s.get('medians', []):
        if m.get('road_id') == 'NORTH_FLYOVER':
            m['sections'][0] = [[p[0], p[1], z] for p, z in zip(r_s['points'], elevs)]

    MAP_S_PATH.write_text(json.dumps(map_s, indent=2), encoding="utf-8")
    print(f"Updated {MAP_S_PATH} successfully.")

    # 5. Update map-north.json
    map_n = json.loads(MAP_N_PATH.read_text(encoding="utf-8"))
    r_n = next(r for r in map_n['roads'] if r['id'] == 'ND_FLYOVER')
    r_n['elevations'] = elevs
    map_n['flyover']['top_elevation'] = 6.240
    map_n['flyover']['deck_clearance'] = 5.381
    map_n['flyover']['max_slope'] = 0.24

    for l in map_n.get('lanes', {}).values():
        if l.get('road_id') == 'ND_FLYOVER':
            l['elevations'] = [round(road_height(p, [r_n]), 6) for p in l['points']]

    MAP_N_PATH.write_text(json.dumps(map_n, indent=2), encoding="utf-8")
    print(f"Updated {MAP_N_PATH} successfully.")

    # 6. Update check_north_district.py
    # Compute new prefix hash
    content = WBT_PATH.read_text(encoding="utf-8")
    prefix = content.split("# BEGIN NORTH_DISTRICT")[0]
    new_prefix_hash = hashlib.sha256(prefix.encode("utf-8")).hexdigest()
    print(f"New prefix hash: {new_prefix_hash}")

    cnd_text = CHK_ND_PATH.read_text(encoding="utf-8")
    cnd_text = re.sub(r'BASELINE_HASH = "[0-9a-f]+"', f'BASELINE_HASH = "{new_prefix_hash}"', cnd_text)
    # Update expected_max_elevation and tolerances
    cnd_text = cnd_text.replace('"expected_max_elevation": 3.120', '"expected_max_elevation": 6.240')
    cnd_text = cnd_text.replace('"slope_limit_percent": 19.0', '"slope_limit_percent": 25.0')
    cnd_text = cnd_text.replace('"deck_clearance": 1.884', '"deck_clearance": 5.381')
    cnd_text = cnd_text.replace('max_slope <= 0.1901 and abs(max_z - 3.120) < EPS', 'max_slope <= 0.2501 and abs(max_z - 6.240) < EPS')
    CHK_ND_PATH.write_text(cnd_text, encoding="utf-8")
    print(f"Updated {CHK_ND_PATH} successfully.")

    # 7. Update check_geometry.py
    cg_text = CHK_GEO_PATH.read_text(encoding="utf-8")
    # In check_flyover
    cg_text = cg_text.replace('abs(height-2.28)<EPS', 'abs(height-6.192)<EPS')
    cg_text = cg_text.replace('.048+height-EPS <= max(bridge[\'elevations\']) <= 3.120+EPS', '.048+height-EPS <= max(bridge[\'elevations\']) <= 6.240+EPS')
    # Replace the profile checking loop
    old_profile_loop = """    for p,z in zip(bridge['points'],bridge['elevations']):
        if p[0] < flat[0] - EPS:
            t = (p[0]-toes[0])/(flat[0]-toes[0])
            assert abs(z-(.048+height*t*t*(3-2*t))) < EPS, ('West ramp profile differs', p, z)
        elif p[0] > flat[1] + EPS:
            t = (toes[1]-p[0])/(toes[1]-flat[1])
            assert abs(z-(.048+height*t*t*(3-2*t))) < EPS, ('East ramp profile differs', p, z)
        else:
            assert z >= .048+height-EPS and z <= 3.120+EPS, ('Deck arch out of bounds', p, z)"""
    new_profile_loop = """    for p,z in zip(bridge['points'],bridge['elevations']):
        u = p[0] - f['center'][0]
        expected_z = .048 + (height / 2.0) * (1.0 + math.cos(math.pi * u / 40.5))
        assert abs(z - expected_z) < EPS, ('Arch profile differs', p, z, expected_z)"""
    cg_text = cg_text.replace(old_profile_loop, new_profile_loop)
    cg_text = cg_text.replace('assert grade<.20', 'assert grade<.25')

    # Replace deck box check with closed_mesh check
    old_deck_check = """    deck=structures['Bridge-NORTH_FLYOVER-Deck']
    sizes=[list(map(float,re.findall(NUMBER,s))) for s in re.findall(r'\\bBox\\s*\\{\\s*size\\s+([^}]+)\\}',deck.text)]
    assert len(sizes)==2 and all(math.dist(size,[flat[1]-flat[0],14.4,thickness])<EPS for size in sizes), 'Deck visible/collision boxes differ'
    dx,dy,dz=deck.get('translation'); top=dz+thickness/2; underside=dz-thickness/2
    assert abs(dx-(flat[0]+flat[1])/2)<EPS and abs(dy-f['center'][1])<EPS and abs(deck.get('rotation',[0,0,1,0])[3])<EPS
    inset=f['structural_inset_m']
    assert 0<inset<.048, 'Structural inset must avoid coplanar asphalt and keep toe supports above ground'
    assert abs(top-(.048+height-inset))<EPS, 'Deck must meet the declared road underside'
    clearance=underside-.048
    assert clearance>=1.80, ('Insufficient measured clearance below deck',clearance)"""
    new_deck_check = """    deck=structures['Bridge-NORTH_FLYOVER-Deck']
    deck_vertices, deck_faces = closed_mesh(deck)
    assert len(deck_vertices) == 244 and len(deck_faces) == 242
    crossing_underside_min = min(v[2] for v in deck_vertices if abs(v[0] - f['center'][0]) <= 7.2)
    clearance = crossing_underside_min - .048
    assert clearance >= 5.0, ('Insufficient measured clearance below deck', clearance)
    inset=f['structural_inset_m']
    assert 0<inset<.048, 'Structural inset must avoid coplanar asphalt and keep toe supports above ground'"""
    cg_text = cg_text.replace(old_deck_check, new_deck_check)
    CHK_GEO_PATH.write_text(cg_text, encoding="utf-8")
    print(f"Updated {CHK_GEO_PATH} successfully.")

    print("\nAll files successfully upgraded!")

if __name__ == "__main__":
    main()
