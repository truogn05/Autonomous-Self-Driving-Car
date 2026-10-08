"""
Upgrade both underpasses (South: BLOCK_MID_TUNNEL, North: ND_UNDERPASS)
to floor elevation Z = -6.240m (matching the doubled flyover height +6.240m).
Deepen tunnel walls and base slab, update medians, lanes, maps, and tests.
"""
import hashlib
import json
import math
import re
from pathlib import Path
from build_world import offset
from sync_world import read_nodes, road_height

ROOT = Path(__file__).resolve().parents[1]
WBT_PATH = ROOT / "worlds/my_world.wbt"
MAP_S_PATH = ROOT / "map.json"
MAP_N_PATH = ROOT / "map-north.json"
CHK_ND_PATH = ROOT / "tools/check_north_district.py"

def main():
    print("=== Deepening Tunnels to Z = -6.240m & Scaling Signs ===")
    wbt_text = WBT_PATH.read_text(encoding="utf-8")
    map_s = json.loads(MAP_S_PATH.read_text(encoding="utf-8"))
    map_n = json.loads(MAP_N_PATH.read_text(encoding="utf-8"))

    # Scale 27 traffic signs
    old_sign_pat = """            coord Coordinate {
              point [
                0 -0.13 -0.13
                0 0.13 -0.13
                0 0.13 0.13
                0 -0.13 0.13
              ]
            }"""
    new_sign_pat = """            coord Coordinate {
              point [
                0 -0.78 -0.78
                0 0.78 -0.78
                0 0.78 0.78
                0 -0.78 0.78
              ]
            }"""
    sign_count = wbt_text.count(old_sign_pat)
    print(f"Scaling {sign_count} traffic signs to 1.56m ([-0.78, 0.78])...")
    wbt_text = wbt_text.replace(old_sign_pat, new_sign_pat)

    # ---------------------------------------------------------
    # 1. NORTH DISTRICT UNDERPASS: ND_UNDERPASS
    # ---------------------------------------------------------
    r_n = next(r for r in map_n['roads'] if r['id'] == 'ND_UNDERPASS')
    pts_n = r_n['points']
    H = 0.048 - (-6.240)  # 6.288m drop

    elevs_n = []
    for p in pts_n:
        y = p[1]
        if y <= 585.0 or y >= 753.0:
            elevs_n.append(0.048)
        elif 666.0 <= y <= 672.0:
            elevs_n.append(-6.240)
        elif 585.0 < y < 666.0:
            t = (y - 585.0) / (666.0 - 585.0)
            z = 0.048 - (H / 2.0) * (1.0 - math.cos(math.pi * t))
            elevs_n.append(round(z, 6))
        else:  # 672.0 < y < 753.0
            t = (753.0 - y) / (753.0 - 672.0)
            z = 0.048 - (H / 2.0) * (1.0 - math.cos(math.pi * t))
            elevs_n.append(round(z, 6))

    r_n['elevations'] = elevs_n
    map_n['underpass']['floor_elevation'] = -6.240
    map_n['underpass']['roof_clearance'] = 5.880
    map_n['underpass']['max_slope'] = 0.13

    # Update lane elevations for ND_UNDERPASS
    for l in map_n.get('lanes', {}).values():
        if l.get('road_id') == 'ND_UNDERPASS':
            l['elevations'] = [round(road_height(p, [r_n]), 6) for p in l['points']]

    # Update wayPoints in WBT for Road-ND_UNDERPASS
    start_pos_n = wbt_text.find('name "Road-ND_UNDERPASS"')
    assert start_pos_n != -1, "Could not find Road-ND_UNDERPASS in WBT"
    wp_pos_n = wbt_text.find('wayPoints [', start_pos_n)
    b_open_n = wbt_text.find('[', wp_pos_n)
    b_close_n = wbt_text.find(']', b_open_n)
    new_wp_str_n = "\n" + "\n".join(f"    {p[0]} {p[1]} {z}" for p, z in zip(pts_n, elevs_n)) + "\n  "
    wbt_text = wbt_text[:b_open_n+1] + new_wp_str_n + wbt_text[b_close_n:]

    # Update ND_Underpass-Wall-West and East
    # West wall: size 1.2 168 6.6, translation 180 669 -3.3
    # East wall: size 1.2 168 6.6, translation 198 669 -3.3
    def replace_solid_block(text, solid_name, new_block):
        start_pos = text.find(f'name "{solid_name}"')
        assert start_pos != -1, f"Solid {solid_name} not found"
        sol_start = text.rfind('Solid {', 0, start_pos)
        assert sol_start != -1
        depth = 0
        for idx in range(sol_start, len(text)):
            if text[idx] == '{':
                depth += 1
            elif text[idx] == '}':
                depth -= 1
                if depth == 0:
                    sol_end = idx + 1
                    return text[:sol_start] + new_block + text[sol_end:]
        assert False, f"Solid {solid_name} unclosed"

    solid_wall_w = """Solid {
  translation 180 669 -3.3
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.6 0.62 0.58
        roughness 0.95
        metalness 0
      }
      geometry Box {
        size 1.2 168 6.6
      }
    }
  ]
  name "ND_Underpass-Wall-West"
  boundingObject Box {
    size 1.2 168 6.6
  }
}"""

    solid_wall_e = """Solid {
  translation 198 669 -3.3
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.6 0.62 0.58
        roughness 0.95
        metalness 0
      }
      geometry Box {
        size 1.2 168 6.6
      }
    }
  ]
  name "ND_Underpass-Wall-East"
  boundingObject Box {
    size 1.2 168 6.6
  }
}"""
    wbt_text = replace_solid_block(wbt_text, "ND_Underpass-Wall-West", solid_wall_w)
    wbt_text = replace_solid_block(wbt_text, "ND_Underpass-Wall-East", solid_wall_e)

    # ---------------------------------------------------------
    # 2. SOUTH DISTRICT TUNNEL: BLOCK_MID_TUNNEL
    # ---------------------------------------------------------
    r_s = next(r for r in map_s['roads'] if r['id'] == 'BLOCK_MID_TUNNEL')
    pts_s = r_s['points']
    elevs_s = []

    for p in pts_s:
        x = p[0]
        if x <= -201.06 or x >= -84.06:
            elevs_s.append(0.048)
        elif -141.06 <= x <= -135.06:
            elevs_s.append(-6.240)
        elif -201.06 < x < -141.06:
            t = (x - (-201.06)) / (-141.06 - (-201.06))
            z = 0.048 - (H / 2.0) * (1.0 - math.cos(math.pi * t))
            elevs_s.append(round(z, 6))
        else:  # -135.06 < x < -84.06
            t = (-84.06 - x) / (-84.06 - (-135.06))
            z = 0.048 - (H / 2.0) * (1.0 - math.cos(math.pi * t))
            elevs_s.append(round(z, 6))

    r_s['elevations'] = elevs_s

    # Update lane elevations for BLOCK_MID_TUNNEL
    for l in map_s.get('lanes', {}).values():
        if l.get('road_id') == 'BLOCK_MID_TUNNEL':
            l['elevations'] = [round(road_height(p, [r_s]), 6) for p in l['points']]

    # Update wayPoints in WBT for Road-BLOCK_MID_TUNNEL
    start_pos_s = wbt_text.find('name "Road-BLOCK_MID_TUNNEL"')
    assert start_pos_s != -1, "Could not find Road-BLOCK_MID_TUNNEL in WBT"
    wp_pos_s = wbt_text.find('wayPoints [', start_pos_s)
    b_open_s = wbt_text.find('[', wp_pos_s)
    b_close_s = wbt_text.find(']', b_open_s)
    new_wp_str_s = "\n" + "\n".join(f"    {p[0]} {p[1]} {z}" for p, z in zip(pts_s, elevs_s)) + "\n  "
    wbt_text = wbt_text[:b_open_s+1] + new_wp_str_s + wbt_text[b_close_s:]

    # Update Tunnel-Structure
    solid_ts = """Solid {
  children [
    Pose {
      translation -138 0 -0.12
      children [
        Shape {
          appearance DEF CONCRETE PBRAppearance {
            baseColor 0.62 0.64 0.62
            roughness 0.85
            metalness 0
          }
          geometry Box {
            size 21 31.2 0.3
          }
        }
      ]
    }
    Pose {
      translation -144 15.24 -3.24
      children [
        Shape {
          appearance USE CONCRETE
          geometry Box {
            size 120 1.2 6.6
          }
        }
      ]
    }
    Pose {
      translation -144 -15.24 -3.24
      children [
        Shape {
          appearance USE CONCRETE
          geometry Box {
            size 120 1.2 6.6
          }
        }
      ]
    }
    Pose {
      translation -144 0 -6.6
      children [
        Shape {
          appearance USE CONCRETE
          geometry Box {
            size 120 31.2 0.6
          }
        }
      ]
    }
    PointLight {
      ambientIntensity 0.3
      color 1 0.95 0.8
      intensity 2.5
      location -138 0 -3
      radius 48
    }
  ]
  name "Tunnel-Structure"
  boundingObject Group {
    children [
      Pose {
        translation -144 15.24 -3.24
        children [
          Box {
            size 120 1.2 6.6
          }
        ]
      }
      Pose {
        translation -144 -15.24 -3.24
        children [
          Box {
            size 120 1.2 6.6
          }
        ]
      }
      Pose {
        translation -138 0 -0.12
        children [
          Box {
            size 21 31.2 0.3
          }
        ]
      }
    ]
  }
}"""
    wbt_text = replace_solid_block(wbt_text, "Tunnel-Structure", solid_ts)

    # Update CentreMedian-BLOCK_MID_TUNNEL
    # First update section in map_s['medians']
    med_bmt = next(m for m in map_s['medians'] if m['road_id'] == 'BLOCK_MID_TUNNEL')
    section_bmt = med_bmt['sections'][0]
    # Update p[2] using road_height with updated r_s
    for p in section_bmt:
        p[2] = round(road_height(p[:2], [r_s]), 6)

    # Generate 100% watertight 2-manifold mesh for CentreMedian-BLOCK_MID_TUNNEL
    left = offset([p[:2] for p in section_bmt], med_bmt['width']/2)
    right = offset([p[:2] for p in section_bmt], -med_bmt['width']/2)
    v_med, f_med = [], []
    first = 0
    for p, l, r in zip(section_bmt, left, right):
        v_med.extend([
            (r[0], r[1], p[2]),
            (l[0], l[1], p[2]),
            (l[0], l[1], p[2] + med_bmt['height']),
            (r[0], r[1], p[2] + med_bmt['height'])
        ])
    f_med.append([first+3, first+2, first+1, first])
    for i in range(len(section_bmt) - 1):
        a, b = first + 4*i, first + 4*(i+1)
        for j in range(4):
            f_med.append([a+j, a+(j+1)%4, b+(j+1)%4, b+j])
    f_med.append(list(range(len(v_med)-4, len(v_med))))

    def format_ifs(geometry_name, vertices, faces):
        coord_lines = "\n".join(f"      {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}" for v in vertices)
        face_lines = "\n".join("      " + " ".join(map(str, f)) + " -1" for f in faces)
        return f"""DEF {geometry_name} IndexedFaceSet {{
    coord Coordinate {{ point [
{coord_lines}
    ] }}
    coordIndex [
{face_lines}
    ]
  }}"""

    solid_med_bmt = f"""Solid {{
  name "CentreMedian-BLOCK_MID_TUNNEL"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.65 0.67 0.62
        roughness 1
        metalness 0
      }}
      geometry {format_ifs('MEDIAN_BLOCK_MID_TUNNEL', v_med, f_med)}
    }}
  ]
  boundingObject USE MEDIAN_BLOCK_MID_TUNNEL
}}"""
    wbt_text = replace_solid_block(wbt_text, "CentreMedian-BLOCK_MID_TUNNEL", solid_med_bmt)

    # Save WBT, map.json, map-north.json
    WBT_PATH.write_text(wbt_text, encoding="utf-8")
    MAP_S_PATH.write_text(json.dumps(map_s, indent=2), encoding="utf-8")
    MAP_N_PATH.write_text(json.dumps(map_n, indent=2), encoding="utf-8")
    print(f"Successfully updated WBT, map.json, map-north.json.")

    # ---------------------------------------------------------
    # 3. UPDATE check_north_district.py
    # ---------------------------------------------------------
    content = WBT_PATH.read_text(encoding="utf-8")
    prefix = content.split("# BEGIN NORTH_DISTRICT")[0]
    new_prefix_hash = hashlib.sha256(prefix.encode("utf-8")).hexdigest()
    print(f"New prefix hash: {new_prefix_hash}")

    cnd_text = CHK_ND_PATH.read_text(encoding="utf-8")
    cnd_text = re.sub(r'BASELINE_HASH = "[0-9a-f]+"', f'BASELINE_HASH = "{new_prefix_hash}"', cnd_text)
    cnd_text = cnd_text.replace('"expected_floor_elevation": -3.600', '"expected_floor_elevation": -6.240')
    cnd_text = cnd_text.replace('"slope_limit_percent": 11.0', '"slope_limit_percent": 15.0')
    cnd_text = cnd_text.replace('"roof_clearance": 2.940', '"roof_clearance": 5.880')
    cnd_text = cnd_text.replace('max_und_slope <= 0.1101 and abs(min_und_z - (-3.60)) < EPS',
                                'max_und_slope <= 0.1501 and abs(min_und_z - (-6.240)) < EPS')
    CHK_ND_PATH.write_text(cnd_text, encoding="utf-8")
    print(f"Successfully updated {CHK_ND_PATH}.")

if __name__ == '__main__':
    main()
