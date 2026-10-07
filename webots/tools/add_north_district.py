"""Generator for the North District expansion of Webots simulation town.
Strictly complies with AGENT.md and audit-07-10-2026-00-02-42.md.
"""
from pathlib import Path
import json
import math
import hashlib
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = ROOT / "worlds" / "my_world.wbt"
CANDIDATE_WORLD_PATH = ROOT / "worlds" / "my_world_north_candidate.wbt"
MAP_NORTH_PATH = ROOT / "map-north.json"
VALIDATION_DIR = ROOT / "validation" / "north-district"

BASELINE_HASH = "43893716767c5eb3e68aa1f289f3386f2da89136b242d25497178550c6614da6"
ASPHALT = 'PBRAppearance { baseColor 0.12 0.14 0.16 roughness 1 metalness 0 }'
PAVEMENT = 'PBRAppearance { baseColor 0.65 0.67 0.62 roughness 1 metalness 0 }'
WATER_APP = 'PBRAppearance { baseColor 0.1 0.35 0.65 roughness 0.2 metalness 0.1 }'
GROUND_APP = 'PBRAppearance { baseColor 0.38 0.48 0.32 roughness 0.85 metalness 0 }'
WALL_APP = 'PBRAppearance { baseColor 0.6 0.62 0.58 roughness 0.95 metalness 0 }'

# --- River & Shore Functions ---
def s_func(x: float) -> float:
    return 51.5 + 2.0 * math.sin(2.0 * math.pi * (x + 52.0) / 104.0)

def w_func(x: float) -> float:
    return 13.0 + 2.0 * math.sin(4.0 * math.pi * (x + 52.0) / 104.0 + math.pi / 3.0)

def n_func(x: float) -> float:
    return s_func(x) + w_func(x)

def s_quintic(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 6.0 * t**5 - 15.0 * t**4 + 10.0 * t**3

def shore_y(x: float) -> float:
    if x <= -40.0:
        return 75.0277
    elif x < -20.6:
        t = (x - (-40.0)) / (-20.6 - (-40.0))
        return 75.0277 + (72.4902 - 75.0277) * s_quintic(t)
    elif x <= -14.4:
        return 72.4902
    elif x < -3.1:
        t = (x - (-14.4)) / (-3.1 - (-14.4))
        return 72.4902 + (74.2321 - 72.4902) * s_quintic(t)
    elif x <= 3.1:
        return 74.2321
    elif x < 14.4:
        t = (x - 3.1) / (14.4 - 3.1)
        return 74.2321 + (70.7177 - 74.2321) * s_quintic(t)
    elif x <= 20.6:
        return 70.7177
    elif x < 28.4:
        t = (x - 20.6) / (28.4 - 20.6)
        return 70.7177 + (68.6292 - 70.7177) * s_quintic(t)
    else:
        return 68.6292

def flyover_z(y: float) -> float:
    if y <= 104.75 or y >= 118.25:
        return 0.008
    elif y < 107.75:
        t = (y - 104.75) / 3.0
        return 0.008 + 0.38 * (3.0 * t**2 - 2.0 * t**3)
    elif y <= 115.25:
        return 0.388
    else:
        t = (118.25 - y) / 3.0
        return 0.008 + 0.38 * (3.0 * t**2 - 2.0 * t**3)

def underpass_z(y: float) -> float:
    if y <= 97.5 or y >= 125.5:
        return 0.008
    elif y < 105.5:
        t = (y - 97.5) / 8.0
        return 0.008 - 0.558 * (3.0 * t**2 - 2.0 * t**3)
    elif y <= 117.5:
        return -0.55
    else:
        t = (125.5 - y) / 8.0
        return 0.008 - 0.558 * (3.0 * t**2 - 2.0 * t**3)

def fmt(v: float) -> str:
    return f"{v:.6f}".rstrip("0").rstrip(".") or "0"

def vec3(p) -> str:
    return f"{fmt(p[0])} {fmt(p[1])} {fmt(p[2])}"

def verify_mesh_manifold(name: str, vertices: list, faces: list):
    for f in faces:
        assert len(f) >= 3, f"{name}: Face with < 3 vertices: {f}"
        assert len(set(f)) == len(f), f"{name}: Duplicate vertices in face: {f}"
    edges = Counter((a, b) for f in faces for a, b in zip(f, f[1:] + f[:1]))
    open_edges = [k for k, count in edges.items() if count != 1 or edges.get((k[1], k[0]), 0) != 1]
    if open_edges:
        raise AssertionError(f"{name}: Mesh is open or inconsistently oriented ({len(open_edges)} bad edges, sample: {open_edges[:5]})")
    used_v = set(v for f in faces for v in f)
    assert used_v == set(range(len(vertices))), f"{name}: Unused vertices: {set(range(len(vertices))) - used_v}"
    return True

def mesh_to_vrml(solid_name: str, def_name: str, appearance: str, vertices: list, faces: list) -> str:
    verify_mesh_manifold(solid_name, vertices, faces)
    pts_str = "\n            ".join(f"{fmt(v[0])} {fmt(v[1])} {fmt(v[2])}" for v in vertices)
    idx_str = "\n            ".join(" ".join(str(idx) for idx in f) + " -1" for f in faces)
    return f"""Solid {{
  translation 0 0 0
  rotation 0 0 1 0
  children [
    Shape {{
      appearance {appearance}
      geometry DEF {def_name} IndexedFaceSet {{
        coord Coordinate {{
          point [
            {pts_str}
          ]
        }}
        coordIndex [
          {idx_str}
        ]
      }}
    }}
  ]
  name "{solid_name}"
  boundingObject USE {def_name}
}}"""

def generate_prism_strip(x_pts: list, y_bot_func, y_top_func, z_top: float, z_bot: float, solid_name: str, def_name: str, appearance: str) -> str:
    vertices = []
    for x in x_pts:
        yb = y_bot_func(x) if callable(y_bot_func) else y_bot_func
        yt = y_top_func(x) if callable(y_top_func) else y_top_func
        vertices.append([round(x, 6), round(yb, 6), round(z_top, 6)])  # 4*i + 0: st
        vertices.append([round(x, 6), round(yb, 6), round(z_bot, 6)])  # 4*i + 1: sb
        vertices.append([round(x, 6), round(yt, 6), round(z_top, 6)])  # 4*i + 2: nt
        vertices.append([round(x, 6), round(yt, 6), round(z_bot, 6)])  # 4*i + 3: nb
    
    n = len(x_pts)
    faces = []
    for i in range(n - 1):
        st_0, sb_0, nt_0, nb_0 = 4*i, 4*i+1, 4*i+2, 4*i+3
        st_1, sb_1, nt_1, nb_1 = 4*(i+1), 4*(i+1)+1, 4*(i+1)+2, 4*(i+1)+3
        
        # Top face (normal +Z)
        faces.append([st_0, st_1, nt_1, nt_0])
        # Bottom face (normal -Z)
        faces.append([sb_0, nb_0, nb_1, sb_1])
        # South face (normal -Y)
        faces.append([sb_0, sb_1, st_1, st_0])
        # North face (normal +Y)
        faces.append([nb_1, nb_0, nt_0, nt_1])
        
    # West cap (i = 0)
    faces.append([1, 0, 2, 3])
    # East cap (i = n-1)
    K = 4 * (n - 1)
    faces.append([K+0, K+1, K+3, K+2])
    
    return mesh_to_vrml(solid_name, def_name, appearance, vertices, faces)

def generate_ramp_support(y_pts: list, x_left: float, x_right: float, z_profile_func, solid_name: str, def_name: str) -> str:
    vertices = []
    for y in y_pts:
        zt = z_profile_func(y) - 0.006
        vertices.append([round(x_left, 6), round(y, 6), round(zt, 6)])    # 4*j + 0: wt
        vertices.append([round(x_left, 6), round(y, 6), 0.0])              # 4*j + 1: wb
        vertices.append([round(x_right, 6), round(y, 6), round(zt, 6)])   # 4*j + 2: et
        vertices.append([round(x_right, 6), round(y, 6), 0.0])             # 4*j + 3: eb
    
    n = len(y_pts)
    faces = []
    for j in range(n - 1):
        wt_0, wb_0, et_0, eb_0 = 4*j, 4*j+1, 4*j+2, 4*j+3
        wt_1, wb_1, et_1, eb_1 = 4*(j+1), 4*(j+1)+1, 4*(j+1)+2, 4*(j+1)+3
        
        # Top
        faces.append([wt_0, wt_1, et_1, et_0])
        # Bottom
        faces.append([wb_0, eb_0, eb_1, wb_1])
        # West
        faces.append([wb_0, wb_1, wt_1, wt_0])
        # East
        faces.append([et_0, et_1, eb_1, eb_0])
        
    # South cap (j = 0)
    faces.append([1, 0, 2, 3])
    # North cap (j = n - 1)
    K = 4 * (n - 1)
    faces.append([K+0, K+1, K+3, K+2])
    
    return mesh_to_vrml(solid_name, def_name, PAVEMENT, vertices, faces)

def generate_box_solid(solid_name: str, def_name: str, translation: tuple, size: tuple, appearance: str) -> str:
    t_str = f"{fmt(translation[0])} {fmt(translation[1])} {fmt(translation[2])}"
    s_str = f"{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}"
    return f"""Solid {{
  translation {t_str}
  children [
    Shape {{
      appearance {appearance}
      geometry Box {{
        size {s_str}
      }}
    }}
  ]
  name "{solid_name}"
  boundingObject Box {{
    size {s_str}
  }}
}}"""

def build_terrain_nodes() -> list:
    nodes = []
    x_steps = [round(-52.0 + i * 0.5, 4) for i in range(209)]
    
    nodes.append(generate_prism_strip(x_steps, 48.0, s_func, 0.0, -0.12, "ND_GROUND_SOUTH_BANK", "ND_MESH_GROUND_SOUTH_BANK", GROUND_APP))
    nodes.append(generate_prism_strip(x_steps, s_func, n_func, -0.15, -0.25, "ND_RIVER_BED", "ND_MESH_RIVER_BED", PAVEMENT))
    nodes.append(generate_prism_strip(x_steps, s_func, n_func, -0.030, -0.150, "ND_RIVER_WATER", "ND_MESH_RIVER_WATER", WATER_APP))
    
    x_west = [round(-52.0 + i * 0.5, 4) for i in range(164)]
    x_west.append(29.9)
    nodes.append(generate_prism_strip(x_west, n_func, 149.0, 0.0, -0.12, "ND_GROUND_NORTH_WEST", "ND_MESH_GROUND_NORTH_WEST", GROUND_APP))
    
    x_east = [33.1] + [round(33.5 + i * 0.5, 4) for i in range(38)]
    nodes.append(generate_prism_strip(x_east, n_func, 149.0, 0.0, -0.12, "ND_GROUND_NORTH_EAST", "ND_MESH_GROUND_NORTH_EAST", GROUND_APP))
    
    x_mid = [29.9, 30.5, 31.5, 32.5, 33.1]
    nodes.append(generate_prism_strip(x_mid, n_func, 97.5, 0.0, -0.12, "ND_GROUND_NORTH_MID_SOUTH", "ND_MESH_GROUND_NORTH_MID_SOUTH", GROUND_APP))
    nodes.append(generate_prism_strip(x_mid, 125.5, 149.0, 0.0, -0.12, "ND_GROUND_NORTH_MID_NORTH", "ND_MESH_GROUND_NORTH_MID_NORTH", GROUND_APP))
    
    nodes.append(generate_box_solid("ND_Bridge-Deck", "ND_BOX_BRIDGE_DECK", (17.5, 111.5, 0.352), (2.4, 7.5, 0.06), PAVEMENT))
    y_ramp_s = [round(104.75 + i * 0.125, 4) for i in range(25)]
    nodes.append(generate_ramp_support(y_ramp_s, 16.3, 18.7, flyover_z, "ND_Bridge-RampSupport-South", "ND_RAMP_SUPPORT_SOUTH"))
    y_ramp_n = [round(115.25 + i * 0.125, 4) for i in range(25)]
    nodes.append(generate_ramp_support(y_ramp_n, 16.3, 18.7, flyover_z, "ND_Bridge-RampSupport-North", "ND_RAMP_SUPPORT_NORTH"))
    
    nodes.append(generate_box_solid("ND_Underpass-Roof", "ND_BOX_UNDERPASS_ROOF", (31.5, 111.5, -0.03), (3.2, 5.6, 0.06), PAVEMENT))
    nodes.append(generate_box_solid("ND_Underpass-Wall-West", "ND_BOX_UNDERPASS_WALL_WEST", (29.85, 111.5, -0.275), (0.1, 28.0, 0.55), WALL_APP))
    nodes.append(generate_box_solid("ND_Underpass-Wall-East", "ND_BOX_UNDERPASS_WALL_EAST", (33.15, 111.5, -0.275), (0.1, 28.0, 0.55), WALL_APP))
    
    return nodes

def build_junctions() -> tuple:
    junc_data = {}
    junc_nodes = []
    
    def crossroad_node(jid: str, name: str, shape_pts: list, road_ids: list) -> str:
        if shape_pts:
            signed_area = sum(shape_pts[i][0] * shape_pts[(i+1)%len(shape_pts)][1] - shape_pts[(i+1)%len(shape_pts)][0] * shape_pts[i][1] for i in range(len(shape_pts)))
            ordered_pts = list(shape_pts) if signed_area > 0 else list(reversed(shape_pts))
            shape_str = "\n    ".join(f"{fmt(-p[1])} {fmt(-p[0])} 0" for p in ordered_pts)
        else:
            shape_str = ""
        roads_str = "\n    ".join(f'"{rid}"' for rid in road_ids)
        return f"""Crossroad {{
  translation 0 0 0.008
  name "{name}"
  id "{jid}"
  shape [
    {shape_str}
  ]
  connectedRoadIDs [
    {roads_str}
  ]
  bottom TRUE
  appearance {ASPHALT}
}}"""

    # 1. WM_T (-45, 111.5)
    jid = "ND_J_WM_T"
    shape = [(-46.2, 109.7), (-46.2, 113.3), (-43.8, 113.3), (-43.2, 112.7), (-43.2, 110.3), (-43.8, 109.7)]
    roads = ["ND_OUTER_NW", "ND_OUTER_W", "ND_ROUND_W"]
    ports = [
        {"edge": "ND_OUTER_NW", "position": [-45.0, 113.3], "outward": [0.0, 1.0], "width": 2.4},
        {"edge": "ND_OUTER_W", "position": [-45.0, 109.7], "outward": [0.0, -1.0], "width": 2.4},
        {"edge": "ND_ROUND_W", "position": [-43.2, 111.5], "outward": [1.0, 0.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [-45.0, 111.5], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-WM_T", shape, roads))

    # 2. WA_T (-45, 95.5)
    jid = "ND_J_WA_T"
    shape = [(-46.2, 93.7), (-46.2, 97.3), (-43.8, 97.3), (-43.2, 96.1), (-43.2, 94.9), (-43.8, 93.7)]
    roads = ["ND_OUTER_W", "ND_OUTER_SW", "ND_LOCAL_W"]
    ports = [
        {"edge": "ND_OUTER_W", "position": [-45.0, 97.3], "outward": [0.0, 1.0], "width": 2.4},
        {"edge": "ND_OUTER_SW", "position": [-45.0, 93.7], "outward": [0.0, -1.0], "width": 2.4},
        {"edge": "ND_LOCAL_W", "position": [-43.2, 95.5], "outward": [1.0, 0.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [-45.0, 95.5], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-WA_T", shape, roads))

    # 3. WS_T (-17.5, 95.5)
    jid = "ND_J_WS_T"
    shape = [(-18.7, 97.3), (-16.3, 97.3), (-16.3, 93.7), (-18.7, 93.7), (-19.3, 94.9), (-19.3, 96.1)]
    roads = ["ND_ROUND_S", "ND_ROUND_SOUTH_LINK", "ND_LOCAL_W"]
    ports = [
        {"edge": "ND_ROUND_S", "position": [-17.5, 97.3], "outward": [0.0, 1.0], "width": 2.4},
        {"edge": "ND_ROUND_SOUTH_LINK", "position": [-17.5, 93.7], "outward": [0.0, -1.0], "width": 2.4},
        {"edge": "ND_LOCAL_W", "position": [-19.3, 95.5], "outward": [-1.0, 0.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [-17.5, 95.5], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-WS_T", shape, roads))

    # 4. SW_T (-17.5, 72.4902)
    jid = "ND_J_SW_T"
    shape = [(-19.3, 71.2902), (-19.3, 73.6902), (-18.7, 74.2902), (-16.3, 74.2902), (-15.7, 73.6902), (-15.7, 71.2902)]
    roads = ["ND_OUTER_SW", "ND_SHORE_W", "ND_ROUND_SOUTH_LINK"]
    ports = [
        {"edge": "ND_OUTER_SW", "position": [-19.3, 72.4902], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_SHORE_W", "position": [-15.7, 72.4902], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_ROUND_SOUTH_LINK", "position": [-17.5, 74.2902], "outward": [0.0, 1.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [-17.5, 72.4902], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-SW_T", shape, roads))

    # 5. SL_T (0, 74.2321)
    jid = "ND_J_SL_T"
    shape = [(-1.8, 73.0321), (-1.8, 75.4321), (-0.6, 76.0321), (0.6, 76.0321), (1.8, 75.4321), (1.8, 73.0321)]
    roads = ["ND_SHORE_W", "ND_SHORE_M", "ND_LOCAL_S"]
    ports = [
        {"edge": "ND_SHORE_W", "position": [-1.8, 74.2321], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_SHORE_M", "position": [1.8, 74.2321], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_LOCAL_S", "position": [0.0, 76.0321], "outward": [0.0, 1.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [0.0, 74.2321], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-SL_T", shape, roads))

    # 6. SC_T (17.5, 70.7177)
    jid = "ND_J_SC_T"
    shape = [(14.9, 69.5177), (14.9, 71.9177), (15.1, 73.3177), (19.9, 73.3177), (20.1, 71.9177), (20.1, 69.5177)]
    roads = ["ND_SHORE_M", "ND_SHORE_E", "ND_MAIN_S"]
    ports = [
        {"edge": "ND_SHORE_M", "position": [14.9, 70.7177], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_SHORE_E", "position": [20.1, 70.7177], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_MAIN_S", "position": [17.5, 73.3177], "outward": [0.0, 1.0], "width": 4.8}
    ]
    junc_data[jid] = {"world_id": jid, "position": [17.5, 70.7177], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-SC_T", shape, roads))

    # 7. ST_T (31.5, 68.6292)
    jid = "ND_J_ST_T"
    shape = [(29.7, 67.4292), (29.7, 69.8292), (30.3, 70.4292), (32.7, 70.4292), (33.3, 69.8292), (33.3, 67.4292)]
    roads = ["ND_SHORE_E", "ND_OUTER_SE", "ND_UNDERPASS"]
    ports = [
        {"edge": "ND_SHORE_E", "position": [29.7, 68.6292], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_OUTER_SE", "position": [33.3, 68.6292], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_UNDERPASS", "position": [31.5, 70.4292], "outward": [0.0, 1.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [31.5, 68.6292], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-ST_T", shape, roads))

    # 8. CM_X (0, 111.5)
    jid = "ND_J_CM_X"
    shape = [(-1.8, 110.3), (-1.8, 112.7), (-0.6, 113.3), (0.6, 113.3), (1.8, 112.7), (1.8, 110.3), (0.6, 109.7), (-0.6, 109.7)]
    roads = ["ND_ROUND_E", "ND_MIDDLE_W", "ND_LOCAL_N", "ND_LOCAL_S"]
    ports = [
        {"edge": "ND_ROUND_E", "position": [-1.8, 111.5], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_MIDDLE_W", "position": [1.8, 111.5], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_LOCAL_N", "position": [0.0, 113.3], "outward": [0.0, 1.0], "width": 1.2},
        {"edge": "ND_LOCAL_S", "position": [0.0, 109.7], "outward": [0.0, -1.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [0.0, 111.5], "kind": "X", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-CM_X", shape, roads))

    # 9. CN_T (0, 143)
    jid = "ND_J_CN_T"
    shape = [(-2.6, 145.4), (2.6, 145.4), (2.6, 140.6), (0.6, 140.4), (-0.6, 140.4), (-2.6, 140.6)]
    roads = ["ND_NORTH_W", "ND_NORTH_M", "ND_LOCAL_N"]
    ports = [
        {"edge": "ND_NORTH_W", "position": [-2.6, 143.0], "outward": [-1.0, 0.0], "width": 4.8},
        {"edge": "ND_NORTH_M", "position": [2.6, 143.0], "outward": [1.0, 0.0], "width": 4.8},
        {"edge": "ND_LOCAL_N", "position": [0.0, 140.4], "outward": [0.0, -1.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [0.0, 143.0], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-CN_T", shape, roads))

    # 10. NW_T (-17.5, 143)
    jid = "ND_J_NW_T"
    shape = [(-20.1, 144.2), (-14.9, 145.4), (-14.9, 140.6), (-16.3, 140.4), (-18.7, 140.4), (-20.1, 141.8)]
    roads = ["ND_OUTER_NW", "ND_NORTH_W", "ND_ROUND_N"]
    ports = [
        {"edge": "ND_OUTER_NW", "position": [-20.1, 143.0], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_NORTH_W", "position": [-14.9, 143.0], "outward": [1.0, 0.0], "width": 4.8},
        {"edge": "ND_ROUND_N", "position": [-17.5, 140.4], "outward": [0.0, -1.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [-17.5, 143.0], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-NW_T", shape, roads))

    # 11. NC_T (17.5, 143)
    jid = "ND_J_NC_T"
    shape = [(14.9, 145.4), (20.1, 144.2), (20.1, 141.8), (19.9, 140.4), (15.1, 140.4), (14.9, 140.6)]
    roads = ["ND_NORTH_M", "ND_NORTH_E", "ND_MAIN_N"]
    ports = [
        {"edge": "ND_NORTH_M", "position": [14.9, 143.0], "outward": [-1.0, 0.0], "width": 4.8},
        {"edge": "ND_NORTH_E", "position": [20.1, 143.0], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_MAIN_N", "position": [17.5, 140.4], "outward": [0.0, -1.0], "width": 4.8}
    ]
    junc_data[jid] = {"world_id": jid, "position": [17.5, 143.0], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-NC_T", shape, roads))

    # 12. NT_T (31.5, 143)
    jid = "ND_J_NT_T"
    shape = [(29.7, 144.2), (33.3, 144.2), (33.3, 141.8), (32.7, 141.2), (30.3, 141.2), (29.7, 141.8)]
    roads = ["ND_NORTH_E", "ND_OUTER_NE", "ND_UNDERPASS"]
    ports = [
        {"edge": "ND_NORTH_E", "position": [29.7, 143.0], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_OUTER_NE", "position": [33.3, 143.0], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_UNDERPASS", "position": [31.5, 141.2], "outward": [0.0, -1.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [31.5, 143.0], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-NT_T", shape, roads))

    # 13. EM_T (45, 111.5)
    jid = "ND_J_EM_T"
    shape = [(43.2, 110.3), (43.2, 112.7), (43.8, 113.3), (46.2, 113.3), (46.2, 109.7), (43.8, 109.7)]
    roads = ["ND_OUTER_NE", "ND_OUTER_SE", "ND_MIDDLE_E"]
    ports = [
        {"edge": "ND_OUTER_NE", "position": [45.0, 113.3], "outward": [0.0, 1.0], "width": 2.4},
        {"edge": "ND_OUTER_SE", "position": [45.0, 109.7], "outward": [0.0, -1.0], "width": 2.4},
        {"edge": "ND_MIDDLE_E", "position": [43.2, 111.5], "outward": [-1.0, 0.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [45.0, 111.5], "kind": "T", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-EM_T", shape, roads))

    # 14. F_GROUND (17.5, 111.5)
    jid = "ND_J_F_GROUND"
    shape = [
        (15.1, 108.0), (15.0, 110.3), (15.0, 112.7), (15.1, 115.0),
        (16.3, 115.0), (18.7, 115.0), (19.9, 115.0), (20.0, 112.7),
        (20.0, 110.3), (19.9, 108.0), (18.7, 108.0), (16.3, 108.0)
    ]
    roads = ["ND_MIDDLE_W", "ND_MIDDLE_E", "ND_GROUND_WS", "ND_GROUND_ES", "ND_GROUND_WN", "ND_GROUND_EN"]
    ports = [
        {"edge": "ND_MIDDLE_W", "position": [15.0, 111.5], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_MIDDLE_E", "position": [20.0, 111.5], "outward": [1.0, 0.0], "width": 2.4},
        {"edge": "ND_GROUND_WS", "position": [15.7, 108.0], "outward": [0.0, -1.0], "width": 1.2},
        {"edge": "ND_GROUND_ES", "position": [19.3, 108.0], "outward": [0.0, -1.0], "width": 1.2},
        {"edge": "ND_GROUND_WN", "position": [15.7, 115.0], "outward": [0.0, 1.0], "width": 1.2},
        {"edge": "ND_GROUND_EN", "position": [19.3, 115.0], "outward": [0.0, 1.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [17.5, 111.5], "kind": "under_bridge", "shape": shape, "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-F_GROUND", shape, roads))

    # 15. ROUND (-17.5, 111.5)
    jid = "ND_ROUND"
    ports = [
        {"edge": "ND_ROUND_W", "position": [-24.7, 111.5], "outward": [-1.0, 0.0], "width": 2.4},
        {"edge": "ND_ROUND_N", "position": [-17.5, 118.7], "outward": [0.0, 1.0], "width": 2.4},
        {"edge": "ND_ROUND_S", "position": [-17.5, 104.3], "outward": [0.0, -1.0], "width": 2.4},
        {"edge": "ND_ROUND_E", "position": [-10.3, 111.5], "outward": [1.0, 0.0], "width": 2.4}
    ]
    junc_data[jid] = {"world_id": jid, "position": [-17.5, 111.5], "kind": "roundabout", "shape": [], "ports": ports}
    roundabout_str = f"""Roundabout {{
  translation -17.5 111.5 0.008
  rotation 0 0 1 -0.7853981633974483
  name "ND_Roundabout"
  id "{jid}"
  connectedRoadIDs [
    "ND_ROUND_W"
    "ND_ROUND_N"
    "ND_ROUND_E"
    "ND_ROUND_S"
  ]
  subdivision 32
  borderWidth 0.1
  borderHeight 0.04
  innerRadius 4.2
  outerRadius 6.0
  numberOfLanes 3
  startRoadsLength [
    1.2
  ]
  startRoadsWidth 2.4
  startRoadsNumberOfLanes 4
  startRoadsStartLine []
  startRoadsLine [
    RoadLine {{
      width 0.025
    }}
    RoadLine {{
      color 0.85 0.75 0.3
      type "double"
      width 0.025
    }}
    RoadLine {{
      width 0.025
    }}
  ]
  lines [
    RoadLine {{
      type "dashed"
      width 0.025
    }}
    RoadLine {{
      type "dashed"
      width 0.025
    }}
  ]
  appearance {ASPHALT}
}}"""
    junc_nodes.append(roundabout_str)

    # 16. SPLIT_S (17.5, 104.75)
    jid = "ND_SPLIT_S"
    roads = ["ND_MAIN_S", "ND_GROUND_WS", "ND_GROUND_ES", "ND_FLYOVER"]
    ports = [
        {"edge": "ND_MAIN_S", "position": [17.5, 104.75], "outward": [0.0, -1.0], "width": 4.8},
        {"edge": "ND_FLYOVER", "position": [17.5, 104.75], "outward": [0.0, 1.0], "width": 2.4},
        {"edge": "ND_GROUND_WS", "position": [15.7, 104.75], "outward": [0.0, 1.0], "width": 1.2},
        {"edge": "ND_GROUND_ES", "position": [19.3, 104.75], "outward": [0.0, 1.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [17.5, 104.75], "kind": "lane_split", "shape": [], "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-SPLIT_S", [], roads))

    # 17. SPLIT_N (17.5, 118.25)
    jid = "ND_SPLIT_N"
    roads = ["ND_MAIN_N", "ND_GROUND_WN", "ND_GROUND_EN", "ND_FLYOVER"]
    ports = [
        {"edge": "ND_MAIN_N", "position": [17.5, 118.25], "outward": [0.0, 1.0], "width": 4.8},
        {"edge": "ND_FLYOVER", "position": [17.5, 118.25], "outward": [0.0, -1.0], "width": 2.4},
        {"edge": "ND_GROUND_WN", "position": [15.7, 118.25], "outward": [0.0, -1.0], "width": 1.2},
        {"edge": "ND_GROUND_EN", "position": [19.3, 118.25], "outward": [0.0, -1.0], "width": 1.2}
    ]
    junc_data[jid] = {"world_id": jid, "position": [17.5, 118.25], "kind": "lane_split", "shape": [], "ports": ports}
    junc_nodes.append(crossroad_node(jid, "ND_Junction-SPLIT_N", [], roads))

    return junc_data, junc_nodes

# --- Build 29 Roads ---
def build_roads() -> tuple:
    road_data = []
    road_nodes = []
    
    def lines_str_for(lanes: int, has_median: bool, is_oneway: bool) -> str:
        if lanes == 8:
            return """[
    RoadLine { width 0.025 }
    RoadLine { width 0.025 }
    RoadLine { width 0.025 }
    RoadLine {
      color 0.85 0.75 0.3
      type "none"
      width 0.025
    }
    RoadLine { width 0.025 }
    RoadLine { width 0.025 }
    RoadLine { width 0.025 }
  ]"""
        elif lanes == 4:
            if has_median:
                return """[
    RoadLine { width 0.025 }
    RoadLine {
      color 0.85 0.75 0.3
      type "none"
      width 0.025
    }
    RoadLine { width 0.025 }
  ]"""
            else:
                return """[
    RoadLine { width 0.025 }
    RoadLine {
      color 0.85 0.75 0.3
      type "double"
      width 0.025
    }
    RoadLine { width 0.025 }
  ]"""
        elif lanes == 2:
            if is_oneway:
                return """[
    RoadLine { width 0.025 }
  ]"""
            else:
                return """[
    RoadLine {
      color 0.85 0.75 0.3
      type "double"
      width 0.025
    }
  ]"""
        return "[]"

    def make_road(rid: str, from_j: str, to_j: str, lanes: int, forward: int, pts: list,
                  has_median: bool = False, is_oneway: bool = False,
                  left_border: bool = True, right_border: bool = True,
                  is_flyover: bool = False, is_underpass: bool = False) -> dict:
        width = round(lanes * 0.6, 2)
        elevations = [p[2] for p in pts]
        xy_pts = [[round(p[0], 6), round(p[1], 6)] for p in pts]
        r_entry = {
            "id": rid,
            "name": f"Road-{rid}",
            "from": from_j,
            "to": to_j,
            "number_of_lanes": lanes,
            "forward_lanes": forward,
            "width": width,
            "points": xy_pts,
            "elevations": elevations
        }
        road_data.append(r_entry)
        
        wp_str = ", ".join(f"{fmt(p[0])} {fmt(p[1])} {fmt(p[2])}" for p in pts)
        l_str = lines_str_for(lanes, has_median, is_oneway)
        
        b_height = "0.1" if is_flyover else "0.035"
        b_width = "[ 0.1 ]" if is_flyover else "[ 0.14 ]"
        road_bo = "TRUE" if (is_flyover or is_underpass) else "FALSE"
        
        node_str = f"""Road {{
  name "Road-{rid}"
  id "{rid}"
  startJunction "{from_j}"
  endJunction "{to_j}"
  width {fmt(width)}
  numberOfLanes {lanes}
  numberOfForwardLanes {forward}
  lines {l_str}
  rightBorder {"TRUE" if right_border else "FALSE"}
  leftBorder {"TRUE" if left_border else "FALSE"}
  roadBorderHeight {b_height}
  roadBorderWidth {b_width}
  wayPoints [
    {wp_str}
  ]
  splineSubdivision -1
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  roadBoundingObject {road_bo}
  rightBorderBoundingObject {"TRUE" if right_border else "FALSE"}
  leftBorderBoundingObject {"TRUE" if left_border else "FALSE"}
}}"""
        road_nodes.append(node_str)
        return r_entry

    # 1. ND_OUTER_NW
    nw_pts = [[-45.0, 113.3, 0.008], [-45.0, 138.0, 0.008]]
    for step in range(1, 10):
        th = math.pi - step * (math.pi / 2.0) / 10.0
        nw_pts.append([-40.0 + 5.0 * math.cos(th), 138.0 + 5.0 * math.sin(th), 0.008])
    nw_pts.append([-40.0, 143.0, 0.008])
    nw_pts.append([-20.1, 143.0, 0.008])
    make_road("ND_OUTER_NW", "ND_J_WM_T", "ND_J_NW_T", 4, 2, nw_pts)

    # 2. ND_OUTER_W
    make_road("ND_OUTER_W", "ND_J_WM_T", "ND_J_WA_T", 4, 2, [[-45.0, 109.7, 0.008], [-45.0, 97.3, 0.008]])

    # 3. ND_OUTER_SW
    sw_pts = [[-45.0, 93.7, 0.008], [-45.0, 80.0277, 0.008]]
    for step in range(1, 10):
        th = math.pi + step * (math.pi / 2.0) / 10.0
        sw_pts.append([-40.0 + 5.0 * math.cos(th), 80.0277 + 5.0 * math.sin(th), 0.008])
    sw_pts.append([-40.0, 75.0277, 0.008])
    x_curr = -39.5
    while x_curr < -19.3:
        sw_pts.append([x_curr, shore_y(x_curr), 0.008])
        x_curr += 0.5
    sw_pts.append([-19.3, 72.4902, 0.008])
    make_road("ND_OUTER_SW", "ND_J_WA_T", "ND_J_SW_T", 4, 2, sw_pts)

    # 4. ND_SHORE_W
    sh_w_pts = [[-15.7, 72.4902, 0.008]]
    x_curr = -15.0
    while x_curr < -1.8:
        sh_w_pts.append([x_curr, shore_y(x_curr), 0.008])
        x_curr += 0.5
    sh_w_pts.append([-1.8, 74.2321, 0.008])
    make_road("ND_SHORE_W", "ND_J_SW_T", "ND_J_SL_T", 4, 2, sh_w_pts)

    # 5. ND_SHORE_M
    sh_m_pts = [[1.8, 74.2321, 0.008]]
    x_curr = 2.5
    while x_curr < 14.9:
        sh_m_pts.append([x_curr, shore_y(x_curr), 0.008])
        x_curr += 0.5
    sh_m_pts.append([14.9, 70.7177, 0.008])
    make_road("ND_SHORE_M", "ND_J_SL_T", "ND_J_SC_T", 4, 2, sh_m_pts)

    # 6. ND_SHORE_E
    sh_e_pts = [[20.1, 70.7177, 0.008]]
    x_curr = 20.5
    while x_curr < 29.7:
        sh_e_pts.append([x_curr, shore_y(x_curr), 0.008])
        x_curr += 0.5
    sh_e_pts.append([29.7, 68.6292, 0.008])
    make_road("ND_SHORE_E", "ND_J_SC_T", "ND_J_ST_T", 4, 2, sh_e_pts)

    # 7. ND_OUTER_SE
    se_pts = [[33.3, 68.6292, 0.008], [40.0, 68.6292, 0.008]]
    for step in range(1, 10):
        th = 1.5 * math.pi + step * (math.pi / 2.0) / 10.0
        se_pts.append([40.0 + 5.0 * math.cos(th), 73.6292 + 5.0 * math.sin(th), 0.008])
    se_pts.append([45.0, 73.6292, 0.008])
    se_pts.append([45.0, 109.7, 0.008])
    make_road("ND_OUTER_SE", "ND_J_ST_T", "ND_J_EM_T", 4, 2, se_pts)

    # 8. ND_OUTER_NE
    ne_pts = [[45.0, 113.3, 0.008], [45.0, 138.0, 0.008]]
    for step in range(1, 10):
        th = 0.0 + step * (math.pi / 2.0) / 10.0
        ne_pts.append([40.0 + 5.0 * math.cos(th), 138.0 + 5.0 * math.sin(th), 0.008])
    ne_pts.append([40.0, 143.0, 0.008])
    ne_pts.append([33.3, 143.0, 0.008])
    make_road("ND_OUTER_NE", "ND_J_EM_T", "ND_J_NT_T", 4, 2, ne_pts)

    # 9. ND_NORTH_E
    make_road("ND_NORTH_E", "ND_J_NT_T", "ND_J_NC_T", 4, 2, [[29.7, 143.0, 0.008], [20.1, 143.0, 0.008]])

    # 10. ND_NORTH_M (8 lanes)
    make_road("ND_NORTH_M", "ND_J_NC_T", "ND_J_CN_T", 8, 4, [[14.9, 143.0, 0.008], [2.6, 143.0, 0.008]], has_median=True)

    # 11. ND_NORTH_W (8 lanes)
    make_road("ND_NORTH_W", "ND_J_CN_T", "ND_J_NW_T", 8, 4, [[-2.6, 143.0, 0.008], [-14.9, 143.0, 0.008]], has_median=True)

    # 12. ND_ROUND_W
    make_road("ND_ROUND_W", "ND_ROUND", "ND_J_WM_T", 4, 2, [[-24.7, 111.5, 0.008], [-43.2, 111.5, 0.008]])

    # 13. ND_ROUND_N
    make_road("ND_ROUND_N", "ND_ROUND", "ND_J_NW_T", 4, 2, [[-17.5, 118.7, 0.008], [-17.5, 140.4, 0.008]])

    # 14. ND_ROUND_S
    make_road("ND_ROUND_S", "ND_ROUND", "ND_J_WS_T", 4, 2, [[-17.5, 104.3, 0.008], [-17.5, 97.3, 0.008]])

    # 15. ND_ROUND_SOUTH_LINK
    make_road("ND_ROUND_SOUTH_LINK", "ND_J_WS_T", "ND_J_SW_T", 4, 2, [[-17.5, 93.7, 0.008], [-17.5, 74.2902, 0.008]])

    # 16. ND_ROUND_E
    make_road("ND_ROUND_E", "ND_ROUND", "ND_J_CM_X", 4, 2, [[-10.3, 111.5, 0.008], [-1.8, 111.5, 0.008]])

    # 17. ND_MIDDLE_W
    make_road("ND_MIDDLE_W", "ND_J_CM_X", "ND_J_F_GROUND", 4, 2, [[1.8, 111.5, 0.008], [15.0, 111.5, 0.008]])

    # 18. ND_MIDDLE_E
    make_road("ND_MIDDLE_E", "ND_J_F_GROUND", "ND_J_EM_T", 4, 2, [[20.0, 111.5, 0.008], [43.2, 111.5, 0.008]])

    # 19. ND_MAIN_S (8 lanes)
    make_road("ND_MAIN_S", "ND_J_SC_T", "ND_SPLIT_S", 8, 4, [[17.5, 73.3177, 0.008], [17.5, 104.75, 0.008]], has_median=True)

    # 20. ND_MAIN_N (8 lanes)
    make_road("ND_MAIN_N", "ND_SPLIT_N", "ND_J_NC_T", 8, 4, [[17.5, 118.25, 0.008], [17.5, 140.4, 0.008]], has_median=True)

    # 21. ND_FLYOVER (4 lanes, bridge)
    fly_pts = []
    y_step = 104.75
    while y_step <= 118.2501:
        fly_pts.append([17.5, y_step, flyover_z(y_step)])
        y_step += 0.125
    make_road("ND_FLYOVER", "ND_SPLIT_S", "ND_SPLIT_N", 4, 2, fly_pts, has_median=True, is_flyover=True)

    # 22. ND_GROUND_WS (2 lanes, oneway South)
    make_road("ND_GROUND_WS", "ND_SPLIT_S", "ND_J_F_GROUND", 2, 0, [[15.7, 104.75, 0.008], [15.7, 108.0, 0.008]],
              is_oneway=True, left_border=True, right_border=False)

    # 23. ND_GROUND_ES (2 lanes, oneway North)
    make_road("ND_GROUND_ES", "ND_SPLIT_S", "ND_J_F_GROUND", 2, 2, [[19.3, 104.75, 0.008], [19.3, 108.0, 0.008]],
              is_oneway=True, left_border=False, right_border=True)

    # 24. ND_GROUND_WN (2 lanes, oneway South)
    make_road("ND_GROUND_WN", "ND_J_F_GROUND", "ND_SPLIT_N", 2, 0, [[15.7, 115.0, 0.008], [15.7, 118.25, 0.008]],
              is_oneway=True, left_border=True, right_border=False)

    # 25. ND_GROUND_EN (2 lanes, oneway North)
    make_road("ND_GROUND_EN", "ND_J_F_GROUND", "ND_SPLIT_N", 2, 2, [[19.3, 115.0, 0.008], [19.3, 118.25, 0.008]],
              is_oneway=True, left_border=False, right_border=True)

    # 26. ND_UNDERPASS (4 lanes)
    und_pts = []
    y_step = 70.4292
    und_pts.append([31.5, y_step, 0.008])
    y_step = 71.0
    while y_step <= 141.2001:
        und_pts.append([31.5, y_step, underpass_z(y_step)])
        y_step += 0.5
    und_pts.append([31.5, 141.2, 0.008])
    make_road("ND_UNDERPASS", "ND_J_ST_T", "ND_J_NT_T", 4, 2, und_pts, is_underpass=True)

    # 27. ND_LOCAL_W (2 lanes, 1+1)
    make_road("ND_LOCAL_W", "ND_J_WA_T", "ND_J_WS_T", 2, 1, [[-43.2, 95.5, 0.008], [-19.3, 95.5, 0.008]])

    # 28. ND_LOCAL_S (2 lanes, 1+1)
    make_road("ND_LOCAL_S", "ND_J_SL_T", "ND_J_CM_X", 2, 1, [[0.0, 76.0321, 0.008], [0.0, 109.7, 0.008]])

    # 29. ND_LOCAL_N (2 lanes, 1+1)
    make_road("ND_LOCAL_N", "ND_J_CM_X", "ND_J_CN_T", 2, 1, [[0.0, 113.3, 0.008], [0.0, 140.4, 0.008]])

    return road_data, road_nodes

# --- Build 5 Median Groups ---
def build_median_nodes() -> list:
    nodes = []
    
    # 1. ND_NORTH_M (along Y=143, X in [2.6, 14.9])
    # Piece 1: [2.6, 3.05] (length 0.45)
    nodes.append(generate_box_solid("ND_Median-NORTH_M-1", "ND_BOX_MEDIAN_NORTH_M_1", (2.825, 143.0, 0.058), (0.45, 0.10, 0.10), PAVEMENT))
    # Gap [3.05, 3.50]
    # Piece 2: [3.50, 14.0] (length 10.5)
    nodes.append(generate_box_solid("ND_Median-NORTH_M-2", "ND_BOX_MEDIAN_NORTH_M_2", (8.75, 143.0, 0.058), (10.5, 0.10, 0.10), PAVEMENT))
    # Gap [14.0, 14.45]
    # Piece 3: [14.45, 14.9] (length 0.45)
    nodes.append(generate_box_solid("ND_Median-NORTH_M-3", "ND_BOX_MEDIAN_NORTH_M_3", (14.675, 143.0, 0.058), (0.45, 0.10, 0.10), PAVEMENT))

    # 2. ND_NORTH_W (along Y=143, X in [-14.9, -2.6])
    # Piece 1: [-14.9, -14.45] (length 0.45)
    nodes.append(generate_box_solid("ND_Median-NORTH_W-1", "ND_BOX_MEDIAN_NORTH_W_1", (-14.675, 143.0, 0.058), (0.45, 0.10, 0.10), PAVEMENT))
    # Gap [-14.45, -14.0]
    # Piece 2: [-14.0, -3.50] (length 10.5)
    nodes.append(generate_box_solid("ND_Median-NORTH_W-2", "ND_BOX_MEDIAN_NORTH_W_2", (-8.75, 143.0, 0.058), (10.5, 0.10, 0.10), PAVEMENT))
    # Gap [-3.50, -3.05]
    # Piece 3: [-3.05, -2.6] (length 0.45)
    nodes.append(generate_box_solid("ND_Median-NORTH_W-3", "ND_BOX_MEDIAN_NORTH_W_3", (-2.825, 143.0, 0.058), (0.45, 0.10, 0.10), PAVEMENT))

    # 3. ND_MAIN_S (along X=17.5, Y in [73.3177, 104.75])
    # Piece 1: [73.3177, 73.8] (length 0.4823)
    nodes.append(generate_box_solid("ND_Median-MAIN_S-1", "ND_BOX_MEDIAN_MAIN_S_1", (17.5, 73.55885, 0.058), (0.10, 0.4823, 0.10), PAVEMENT))
    # Gap [73.8, 74.25]
    # Piece 2: [74.25, 104.75] (length 30.5)
    nodes.append(generate_box_solid("ND_Median-MAIN_S-2", "ND_BOX_MEDIAN_MAIN_S_2", (17.5, 89.5, 0.058), (0.10, 30.5, 0.10), PAVEMENT))

    # 4. ND_MAIN_N (along X=17.5, Y in [118.25, 140.4])
    # Piece 1: [118.25, 139.45] (length 21.2)
    nodes.append(generate_box_solid("ND_Median-MAIN_N-1", "ND_BOX_MEDIAN_MAIN_N_1", (17.5, 128.85, 0.058), (0.10, 21.2, 0.10), PAVEMENT))
    # Gap [139.45, 139.9]
    # Piece 2: [139.9, 140.4] (length 0.5)
    nodes.append(generate_box_solid("ND_Median-MAIN_N-2", "ND_BOX_MEDIAN_MAIN_N_2", (17.5, 140.15, 0.058), (0.10, 0.5, 0.10), PAVEMENT))

    # 5. ND_FLYOVER (along X=17.5, Y in [104.75, 118.25], follows 3D profile)
    y_med = [round(104.75 + i * 0.125, 4) for i in range(109)]  # 104.75 to 118.25
    vertices = []
    faces = []
    for y in y_med:
        zb = flyover_z(y)
        zt = zb + 0.10
        vertices.append([17.45, round(y, 6), round(zt, 6)])  # 4*j + 0: wt
        vertices.append([17.45, round(y, 6), round(zb, 6)])  # 4*j + 1: wb
        vertices.append([17.55, round(y, 6), round(zt, 6)])  # 4*j + 2: et
        vertices.append([17.55, round(y, 6), round(zb, 6)])  # 4*j + 3: eb
    n = len(y_med)
    for j in range(n - 1):
        wt_0, wb_0, et_0, eb_0 = 4*j, 4*j+1, 4*j+2, 4*j+3
        wt_1, wb_1, et_1, eb_1 = 4*(j+1), 4*(j+1)+1, 4*(j+1)+2, 4*(j+1)+3
        # Top
        faces.append([wt_0, wt_1, et_1, et_0])
        # Bottom
        faces.append([wb_0, eb_0, eb_1, wb_1])
        # West
        faces.append([wb_0, wb_1, wt_1, wt_0])
        # East
        faces.append([et_0, et_1, eb_1, eb_0])
    # South cap (j = 0)
    faces.append([1, 0, 2, 3])
    # North cap (j = n - 1)
    K = 4 * (n - 1)
    faces.append([K+0, K+1, K+3, K+2])
    nodes.append(mesh_to_vrml("ND_Median-FLYOVER", "ND_MESH_MEDIAN_FLYOVER", PAVEMENT, vertices, faces))

    return nodes

# --- Main Assembler & Exporter ---
def build_north_district_data():
    terrain_nodes = build_terrain_nodes()
    junc_data, junc_nodes = build_junctions()
    road_data, road_nodes = build_roads()
    median_nodes = build_median_nodes()
    
    # Assemble full block
    all_nodes = terrain_nodes + junc_nodes + road_nodes + median_nodes
    block = "# BEGIN NORTH_DISTRICT\n\n" + "\n\n".join(all_nodes) + "\n\n# END NORTH_DISTRICT\n"
    
    map_north = {
        "units": "metres",
        "coordinate_system": "ENU",
        "traffic_side": "right",
        "bounds": [-52, 52, 48, 149],
        "river": {
            "s_formula": "51.5 + 2.0 * sin(2*pi*(x+52)/104)",
            "w_formula": "13.0 + 2.0 * sin(4*pi*(x+52)/104 + pi/3)",
            "water_elevation": -0.030,
            "bed_elevation": -0.150
        },
        "flyover": {
            "span_y": [104.75, 118.25],
            "plateau_y": [107.75, 115.25],
            "max_slope": 0.19,
            "top_elevation": 0.388,
            "deck_clearance": 0.314
        },
        "underpass": {
            "span_y": [97.5, 125.5],
            "floor_y": [105.5, 117.5],
            "max_slope": 0.104625,
            "floor_elevation": -0.550,
            "roof_clearance": 0.490
        },
        "junctions": junc_data,
        "roads": road_data
    }
    return block, map_north

def get_baseline_bytes() -> bytes:
    content = WORLD_PATH.read_bytes()
    if b"# BEGIN NORTH_DISTRICT" in content:
        prefix = content.split(b"# BEGIN NORTH_DISTRICT")[0]
    else:
        prefix = content
    h = hashlib.sha256(prefix).hexdigest()
    if h != BASELINE_HASH:
        import subprocess
        res = subprocess.run(["git", "show", "5fa48e7:webots/worlds/my_world.wbt"], capture_output=True, cwd=ROOT)
        if hashlib.sha256(res.stdout).hexdigest() == BASELINE_HASH:
            return res.stdout
    assert h == BASELINE_HASH, f"World baseline hash mismatch: {h} vs {BASELINE_HASH}"
    return prefix

def generate_candidate():
    source_bytes = get_baseline_bytes()
    block, map_north = build_north_district_data()
    MAP_NORTH_PATH.write_text(json.dumps(map_north, indent=2), encoding="utf-8")
    
    candidate_content = source_bytes.decode("utf-8") + block
    CANDIDATE_WORLD_PATH.write_text(candidate_content, encoding="utf-8", newline="\n")
    print(f"Candidate world generated at {CANDIDATE_WORLD_PATH}")
    print(f"Map north JSON generated at {MAP_NORTH_PATH}")
    return candidate_content

def apply_to_world():
    source_bytes = get_baseline_bytes()
    block, map_north = build_north_district_data()
    MAP_NORTH_PATH.write_text(json.dumps(map_north, indent=2), encoding="utf-8")
    
    new_world_content = source_bytes.decode("utf-8") + block
    WORLD_PATH.write_text(new_world_content, encoding="utf-8", newline="\n")
    print(f"North District block successfully appended to {WORLD_PATH}")

if __name__ == "__main__":
    generate_candidate()
    apply_to_world()

