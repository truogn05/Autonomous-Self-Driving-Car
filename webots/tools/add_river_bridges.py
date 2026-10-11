"""Add 2 River-Crossing Bridges (4 lanes, column center medians, 1.50m high side parapets),
widen the 4 approach corridors to 8 lanes (4 inner bridge lanes + 4 outer ground lanes),
delete N2_CROSS_W, N2_CROSS_E, NORTH_OUTLET_M, and J_N2_W/M/E Crossroads,
and straighten all 3 North-South corridors (West, Middle, East) into continuous straight lines.

Complies strictly with AGENT.md and GEMINI.md.
"""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import build_world as g
from sync_world import read_nodes, mouth, road_height

WBT_PATH = ROOT / "worlds" / "my_world.wbt"
MAP_S_PATH = ROOT / "map.json"
MAP_N_PATH = ROOT / "map-north.json"
CHK_GEO_PATH = ROOT / "tools" / "check_geometry.py"
CHK_ND_PATH = ROOT / "tools" / "check_north_district.py"

ASPHALT = "PBRAppearance { baseColor 0.12 0.14 0.16 roughness 1 metalness 0 }"
PAVEMENT = "PBRAppearance { baseColor 0.65 0.67 0.62 roughness 1 metalness 0 }"
PARAPET_APP = "PBRAppearance { baseColor 0.54 0.58 0.62 roughness 0.75 metalness 0.1 }"
PIER_APP = "PBRAppearance { baseColor 0.58 0.60 0.58 roughness 0.9 metalness 0 }"
COL_BODY_APP = "PBRAppearance { baseColor 0.26 0.32 0.38 roughness 0.55 metalness 0.35 }"
COL_BAND_APP = "PBRAppearance { baseColor 0.96 0.65 0.10 roughness 0.35 metalness 0.15 }"


def fmt(val: float) -> str:
    if abs(val) < 1e-12:
        return "0"
    res = f"{val:.6f}".rstrip("0").rstrip(".")
    return res if res else "0"


def vec(p) -> str:
    return " ".join(fmt(v) for v in p)


def direction_2d(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-9 else (0.0, 1.0)


def pt_along(p0, u, n_left, s, lat=0.0):
    return [round(p0[0] + s * u[0] + lat * n_left[0], 6), round(p0[1] + s * u[1] + lat * n_left[1], 6)]


def verify_mesh_manifold(name: str, vertices: list, faces: list):
    for f in faces:
        assert len(f) >= 3 and len(set(f)) == len(f), f"{name}: Invalid face {f}"
    edges = Counter((a, b) for f in faces for a, b in zip(f, f[1:] + f[:1]))
    open_edges = [k for k, count in edges.items() if count != 1 or edges.get((k[1], k[0]), 0) != 1]
    assert not open_edges, f"{name}: Mesh not 2-manifold ({len(open_edges)} bad edges)"
    used_v = set(v for f in faces for v in f)
    assert used_v == set(range(len(vertices))), f"{name}: Unused vertices"
    vol = 0.0
    for f in faces:
        v0 = vertices[f[0]]
        for k in range(1, len(f) - 1):
            v1, v2 = vertices[f[k]], vertices[f[k + 1]]
            cx = v1[1] * v2[2] - v1[2] * v2[1]
            cy = v1[2] * v2[0] - v1[0] * v2[2]
            cz = v1[0] * v2[1] - v1[1] * v2[0]
            vol += (v0[0] * cx + v0[1] * cy + v0[2] * cz) / 6.0
    assert vol > 0.0, f"{name}: Inverted normals (vol={vol:.4f})"


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


def build_curved_ribbon_mesh(pts_3d, t_west, t_east, z_bot_fn, z_top_fn):
    n = len(pts_3d)
    vertices = []
    for j in range(n):
        p_prev = pts_3d[max(0, j - 1)]
        p_next = pts_3d[min(n - 1, j + 1)]
        ux, uy = direction_2d(p_prev, p_next)
        rx, ry = uy, -ux
        x, y, z = pts_3d[j]
        xw, yw = round(x + t_west * rx, 6), round(y + t_west * ry, 6)
        xe, ye = round(x + t_east * rx, 6), round(y + t_east * ry, 6)
        zb = round(z_bot_fn(j, z), 6)
        zt = round(z_top_fn(j, z), 6)
        assert zt > zb + 1e-4, f"Station {j}: zt ({zt}) <= zb ({zb})"
        vertices.append([xw, yw, zt])  # 4*j + 0: wt
        vertices.append([xw, yw, zb])  # 4*j + 1: wb
        vertices.append([xe, ye, zt])  # 4*j + 2: et
        vertices.append([xe, ye, zb])  # 4*j + 3: eb

    faces = [[1, 3, 2, 0]]
    for j in range(n - 1):
        wt0, wb0, et0, eb0 = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3
        wt1, wb1, et1, eb1 = 4 * (j + 1), 4 * (j + 1) + 1, 4 * (j + 1) + 2, 4 * (j + 1) + 3
        faces.append([wt0, et0, et1, wt1])
        faces.append([wb0, wb1, eb1, eb0])
        faces.append([wb0, wt0, wt1, wb1])
        faces.append([eb0, eb1, et1, et0])
    K = 4 * (n - 1)
    faces.append([K + 1, K + 0, K + 2, K + 3])
    return vertices, faces


def smooth_ramp_z(t: float) -> float:
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    if t < 0.2:
        return 3.125 * t * t
    if t <= 0.8:
        return 0.125 + 1.25 * (t - 0.2)
    u = 1.0 - t
    return 1.0 - 3.125 * u * u


def hermite_xy(p0, p1, m0, m1, t):
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1
    h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2
    h11 = t ** 3 - t ** 2
    x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
    y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
    return [x, y]


def make_bridge_waypoints(p_start, u_start, s_ramp_s, p_north_ramp_start, p_end, n_south=8, n_mid=50, n_north=8):
    """Build 3D waypoints for a river bridge whose South ramp is collinear with u_start
    and whose North ramp is collinear with (0, 1).
    """
    z_road = 0.048
    z_plat = 5.648
    z_arch = 1.852
    pts = []
    # Segment 1: straight South ramp along u_start (length s_ramp_s)
    for i in range(n_south):
        t = i / n_south
        s = t * s_ramp_s
        x = p_start[0] + s * u_start[0]
        y = p_start[1] + s * u_start[1]
        z = z_road + (z_plat - z_road) * smooth_ramp_z(t)
        pts.append([round(x, 6), round(y, 6), round(z, 6)])

    p_curve_0 = [p_start[0] + s_ramp_s * u_start[0], p_start[1] + s_ramp_s * u_start[1]]
    p_curve_1 = p_north_ramp_start
    dist_c = math.dist(p_curve_0, p_curve_1)
    m0 = [u_start[0] * dist_c * 0.85, u_start[1] * dist_c * 0.85]
    m1 = [0.0, 1.0 * dist_c * 0.85]

    # Segment 2: elevated curved river span (j = n_south .. n_south + n_mid)
    for i in range(n_mid + 1):
        t = i / n_mid
        x, y = hermite_xy(p_curve_0, p_curve_1, m0, m1, t)
        z = z_plat + z_arch * math.sin(math.pi * t)
        pts.append([round(x, 6), round(y, 6), round(z, 6)])

    # Segment 3: straight North ramp from p_north_ramp_start to p_end along (0, 1)
    for i in range(1, n_north + 1):
        t = i / n_north
        x = p_north_ramp_start[0] + t * (p_end[0] - p_north_ramp_start[0])
        y = p_north_ramp_start[1] + t * (p_end[1] - p_north_ramp_start[1])
        z = z_road + (z_plat - z_road) * smooth_ramp_z(1.0 - t)
        pts.append([round(x, 6), round(y, 6), round(z, 6)])
    return pts


def build_column_median_solid(solid_name: str, pts_3d: list) -> str:
    """Build center median columns spaced along the 3D centerline of a river bridge."""
    shape_poses = []
    bo_poses = []
    for j in range(2, len(pts_3d) - 2, 2):
        x, y, z = pts_3d[j]
        zc_body = round(z + 0.14 + 0.55, 6)
        zc_band = round(z + 0.14 + 0.90, 6)
        shape_poses.append(f"""    Pose {{
      translation {fmt(x)} {fmt(y)} {fmt(zc_body)}
      children [
        Shape {{
          appearance {COL_BODY_APP}
          geometry Cylinder {{
            height 1.1
            radius 0.18
          }}
        }}
      ]
    }}
    Pose {{
      translation {fmt(x)} {fmt(y)} {fmt(zc_band)}
      children [
        Shape {{
          appearance {COL_BAND_APP}
          geometry Cylinder {{
            height 0.24
            radius 0.19
          }}
        }}
      ]
    }}""")
        bo_poses.append(f"""      Pose {{
        translation {fmt(x)} {fmt(y)} {fmt(zc_body)}
        children [
          Cylinder {{
            height 1.1
            radius 0.18
          }}
        ]
      }}""")
    return f"""Solid {{
  children [
{chr(10).join(shape_poses)}
  ]
  name "{solid_name}"
  boundingObject Group {{
    children [
{chr(10).join(bo_poses)}
    ]
  }}
}}"""


def build_bridge_piers_solid(solid_name: str, pts_3d: list, pier_indices_and_zbot: list) -> str:
    """Build twin cylindrical concrete piers supporting the river bridge deck."""
    shape_poses = []
    bo_poses = []
    n = len(pts_3d)
    for j, z_bot in pier_indices_and_zbot:
        p_prev = pts_3d[max(0, j - 1)]
        p_next = pts_3d[min(n - 1, j + 1)]
        ux, uy = direction_2d(p_prev, p_next)
        rx, ry = uy, -ux
        x, y, z = pts_3d[j]
        z_top = z - 0.42
        h = round(z_top - z_bot, 6)
        zc = round(0.5 * (z_top + z_bot), 6)
        for t_lat in (-4.8, 4.8):
            px, py = round(x + t_lat * rx, 6), round(y + t_lat * ry, 6)
            shape_poses.append(f"""    Pose {{
      translation {fmt(px)} {fmt(py)} {fmt(zc)}
      children [
        Shape {{
          appearance {PIER_APP}
          geometry Cylinder {{
            height {fmt(h)}
            radius 0.85
          }}
        }}
      ]
    }}""")
            bo_poses.append(f"""      Pose {{
        translation {fmt(px)} {fmt(py)} {fmt(zc)}
        children [
          Cylinder {{
            height {fmt(h)}
            radius 0.85
          }}
        ]
      }}""")
    return f"""Solid {{
  children [
{chr(10).join(shape_poses)}
  ]
  name "{solid_name}"
  boundingObject Group {{
    children [
{chr(10).join(bo_poses)}
    ]
  }}
}}"""


def build_south_median_solid(median: dict) -> str:
    vertices, faces = [], []
    for section in median["sections"]:
        left = g.offset([p[:2] for p in section], median["width"] / 2)
        right = g.offset([p[:2] for p in section], -median["width"] / 2)
        first = len(vertices)
        for p, l, r in zip(section, left, right):
            vertices.extend([(*r, p[2]), (*l, p[2]), (*l, p[2] + median["height"]), (*r, p[2] + median["height"])])
        faces.append([first + 3, first + 2, first + 1, first])
        for i in range(len(section) - 1):
            a, b = first + 4 * i, first + 4 * (i + 1)
            for j in range(4):
                faces.append([a + j, a + (j + 1) % 4, b + (j + 1) % 4, b + j])
        faces.append(list(range(len(vertices) - 4, len(vertices))))
    geometry = "MEDIAN_" + median["road_id"]
    pts_str = "\n            ".join(vec(p) for p in vertices)
    idx_str = "\n            ".join(" ".join(map(str, face)) + " -1" for face in faces)
    return f"""Solid {{
  children [
    Shape {{
      appearance {PAVEMENT}
      geometry DEF {geometry} IndexedFaceSet {{
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
  name "CentreMedian-{median['road_id']}"
  boundingObject USE {geometry}
}}"""


def build_south_curb_solid(nid: str, n: dict) -> str:
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union
    boundary = Polygon(n["polygon"]).boundary
    cap_lines = [LineString(mouth(p["position"], p["outward"], p["width"])).buffer(0.0002) for p in n["ports"]]
    caps = unary_union(cap_lines)
    parts = boundary.difference(caps)
    parts = [parts] if parts.geom_type == "LineString" else list(parts.geoms)
    shape_poses, bo_poses = [], []
    for part in parts:
        for a, b in zip(part.coords, list(part.coords)[1:]):
            length = math.dist(a, b)
            if length < 0.006:
                continue
            u = g.direction(a, b)
            center = g.add(g.mul(g.add(a, b), 0.5), g.mul(g.normal(u), -0.42))
            rot_angle = math.atan2(u[1], u[0])
            shape_poses.append(f"""    Pose {{
      translation {vec((*center, 0.153))}
      rotation 0 0 1 {fmt(rot_angle)}
      children [
        Shape {{
          appearance {PAVEMENT}
          geometry Box {{
            size {vec((length, 0.84, 0.21))}
          }}
        }}
      ]
    }}""")
            bo_poses.append(f"""      Pose {{
        translation {vec((*center, 0.153))}
        rotation 0 0 1 {fmt(rot_angle)}
        children [
          Box {{
            size {vec((length, 0.84, 0.21))}
          }}
        ]
      }}""")
    return f"""Solid {{
  children [
{chr(10).join(shape_poses)}
  ]
  name "JunctionCurbs-{nid}"
  boundingObject Group {{
    children [
{chr(10).join(bo_poses)}
    ]
  }}
}}"""


def generate_lanes_for_road_6x(e, lane_graph):
    pts, name = e["points"], e["id"]
    count, forward = e["number_of_lanes"], e["forward_lanes"]
    lane_width = 3.6
    for reverse in (False, True):
        seq = list(reversed(pts)) if reverse else pts
        direction_count = count - forward if reverse else forward
        for index in range(1, direction_count + 1):
            lid = name + ("_B" if reverse else "_F") + str(index)
            lane_graph[lid] = {
                "from": e["to"] if reverse else e["from"],
                "to": e["from"] if reverse else e["to"],
                "points": [[round(x, 6), round(y, 6)] for x, y in g.offset(seq, -e["width"] / 2 + lane_width * (direction_count - index + 0.5))],
                "lane_index": index,
                "road_id": e["id"],
                "direction_lanes": direction_count,
                "successors": [],
                "speed_limit_m_s": 0.5,
            }
    for suffix in ("F", "B"):
        zone = "CHANGE_" + name + "_" + suffix
        for index in range(1, (forward if suffix == "F" else count - forward) + 1):
            lid = name + "_" + suffix + str(index)
            lane = lane_graph.pop(lid)
            points = lane["points"]
            total = g.length(points)
            gap = min(4.8, total / 4.0)
            lane_graph[lid + "_entry"] = {
                **lane,
                "successors": [],
                "to": zone,
                "points": [[round(x, 6), round(y, 6)] for x, y in g.cut(points, 0, total / 2 + gap)],
            }
            lane_graph[lid + "_exit"] = {
                **lane,
                "successors": [],
                "from": zone,
                "points": [[round(x, 6), round(y, 6)] for x, y in g.cut(points, total / 2 + gap, 0)],
            }


def offset_3d(pts, dist):
    res_xy, res_z = [], []
    n = len(pts)
    for i in range(n):
        p_prev = pts[max(0, i - 1)]
        p_next = pts[min(n - 1, i + 1)]
        ux, uy = direction_2d(p_prev, p_next)
        nx, ny = -uy, ux
        p = pts[i]
        res_xy.append([round(p[0] + nx * dist, 6), round(p[1] + ny * dist, 6)])
        res_z.append(round(p[2], 6))
    return res_xy, res_z


def generate_north_lanes(roads: list) -> dict:
    lanes = {}
    for r in roads:
        rid = r["id"]
        pts = [[r["points"][i][0], r["points"][i][1], r["elevations"][i]] for i in range(len(r["points"]))]
        width = r["width"]
        total_lanes = r["number_of_lanes"]
        forward = r["forward_lanes"]
        if forward > 0:
            for idx in range(1, forward + 1):
                lid = f"{rid}_F{idx}"
                d = -width / 2.0 + 3.6 * (forward - idx + 0.5)
                xy, z = offset_3d(pts, d)
                lanes[lid] = {
                    "from": r["from"],
                    "to": r["to"],
                    "points": xy,
                    "elevations": z,
                    "lane_index": idx,
                    "road_id": rid,
                    "direction_lanes": forward,
                    "speed_limit_m_s": 0.5,
                    "successors": [],
                }
        rev_count = total_lanes - forward
        if rev_count > 0:
            pts_rev = list(reversed(pts))
            for idx in range(1, rev_count + 1):
                lid = f"{rid}_B{idx}"
                d = -width / 2.0 + 3.6 * (rev_count - idx + 0.5)
                xy, z = offset_3d(pts_rev, d)
                lanes[lid] = {
                    "from": r["to"],
                    "to": r["from"],
                    "points": xy,
                    "elevations": z,
                    "lane_index": idx,
                    "road_id": rid,
                    "direction_lanes": rev_count,
                    "speed_limit_m_s": 0.5,
                    "successors": [],
                }
    cx, cy = -105.0, 669.0
    sectors = [
        ("E_N", 0.0, math.pi / 2.0, "ND_ROUND_E", "ND_ROUND_N"),
        ("N_W", math.pi / 2.0, math.pi, "ND_ROUND_N", "ND_ROUND_W"),
        ("W_S", math.pi, 3.0 * math.pi / 2.0, "ND_ROUND_W", "ND_ROUND_S"),
        ("S_E", 3.0 * math.pi / 2.0, 2.0 * math.pi, "ND_ROUND_S", "ND_ROUND_E"),
    ]
    for sec_name, a_start, a_end, from_port, to_port in sectors:
        for ring_idx, radius in enumerate([34.2, 30.6, 27.0], start=1):
            lid = f"ND_RING_{sec_name}_L{ring_idx}"
            pts_arc, z_arc = [], []
            steps = 8
            for s in range(steps + 1):
                ang = a_start + s * (a_end - a_start) / steps
                pts_arc.append([round(cx + radius * math.cos(ang), 6), round(cy + radius * math.sin(ang), 6)])
                z_arc.append(0.048)
            lanes[lid] = {
                "from": from_port,
                "to": to_port,
                "points": pts_arc,
                "elevations": z_arc,
                "lane_index": ring_idx,
                "road_id": "ND_ROUND",
                "direction_lanes": 3,
                "speed_limit_m_s": 0.35,
                "successors": [],
            }
    next_sec = {"E_N": "N_W", "N_W": "W_S", "W_S": "S_E", "S_E": "E_N"}
    for sec_name, _, _, _, _ in sectors:
        for ring_idx in (1, 2, 3):
            lanes[f"ND_RING_{sec_name}_L{ring_idx}"]["successors"].append(f"ND_RING_{next_sec[sec_name]}_L{ring_idx}")

    by_dest, by_orig = {}, {}
    for lid, l in lanes.items():
        if l["road_id"] == "ND_ROUND":
            continue
        by_dest.setdefault(l["to"], []).append(lid)
        by_orig.setdefault(l["from"], []).append(lid)

    for jid, incoming in by_dest.items():
        outgoing = by_orig.get(jid, [])
        for in_lid in incoming:
            in_road = lanes[in_lid]["road_id"]
            for out_lid in outgoing:
                out_road = lanes[out_lid]["road_id"]
                if in_road != out_road or jid in ("ND_SPLIT_RIVER_ES", "ND_SPLIT_RIVER_WS"):
                    lanes[in_lid]["successors"].append(out_lid)
    return lanes


def main():
    import subprocess
    repo_root = ROOT.parent
    wbt_full = subprocess.check_output(["git", "show", "7f64b2c:webots/worlds/my_world.wbt"], cwd=repo_root).decode("utf-8")
    assert "# BEGIN NORTH_DISTRICT" in wbt_full and "# END NORTH_DISTRICT" in wbt_full
    prefix_str, rest_str = wbt_full.split("# BEGIN NORTH_DISTRICT", 1)
    nd_str, suffix_str = rest_str.split("# END NORTH_DISTRICT", 1)

    map_s = json.loads(subprocess.check_output(["git", "show", "7f64b2c:webots/map.json"], cwd=repo_root).decode("utf-8"))
    map_n = json.loads(subprocess.check_output(["git", "show", "7f64b2c:webots/map-north.json"], cwd=repo_root).decode("utf-8"))

    # =========================================================================
    # PART 1: UPDATE SOUTH DISTRICT (map.json & prefix_str)
    # =========================================================================
    roads_s = {r["id"]: r for r in map_s["roads"]}
    juncs_s = map_s["junctions"]

    # 1. Compute Straight Corridor Axes (West, Middle, East)
    # West Axis: NORTH_WEST (-72, 129) -> O_N_W (-97.5, 240)
    p_nw = [-72.0, 129.0]
    p_onw = [-97.5, 240.0]
    L_w = math.dist(p_nw, p_onw)
    u_w = direction_2d(p_nw, p_onw)
    n_w = (-u_w[1], u_w[0])
    s_w_start, s_w_split = 19.8, 72.0
    s_ww_end, s_we_end = L_w - 9.5, L_w - 14.0
    pt_w_start = pt_along(p_nw, u_w, n_w, s_w_start, 0.0)
    pt_w_split = pt_along(p_nw, u_w, n_w, s_w_split, 0.0)
    pt_ww_start = pt_along(p_nw, u_w, n_w, s_w_split, +10.8)
    pt_ww_end = pt_along(p_nw, u_w, n_w, s_ww_end, +10.8)
    pt_we_start = pt_along(p_nw, u_w, n_w, s_w_split, -10.8)
    pt_we_end = pt_along(p_nw, u_w, n_w, s_we_end, -10.8)

    # Middle Axis: NORTH_MIDDLE (42, 129) -> O_N_M (18, 240)
    p_nm = [42.0, 129.0]
    p_onm = [18.0, 240.0]
    L_m = math.dist(p_nm, p_onm)
    u_m = direction_2d(p_nm, p_onm)
    n_m = (-u_m[1], u_m[0])
    s_m_start, s_m_end = 24.5, L_m - 12.0
    pt_m_start = pt_along(p_nm, u_m, n_m, s_m_start, 0.0)
    pt_m_end = pt_along(p_nm, u_m, n_m, s_m_end, 0.0)

    # East Axis: NORTH_EAST (132, 129) -> O_N_E (144, 240)
    p_ne = [132.0, 129.0]
    p_one = [144.0, 240.0]
    L_e = math.dist(p_ne, p_one)
    u_e = direction_2d(p_ne, p_one)
    n_e = (-u_e[1], u_e[0])
    s_e_start, s_e_split = 19.5, 71.0
    s_ew_end, s_ee_end = L_e - 12.5, L_e - 10.5
    pt_e_start = pt_along(p_ne, u_e, n_e, s_e_start, 0.0)
    pt_e_split = pt_along(p_ne, u_e, n_e, s_e_split, 0.0)
    pt_ew_start = pt_along(p_ne, u_e, n_e, s_e_split, +10.8)
    pt_ew_end = pt_along(p_ne, u_e, n_e, s_ew_end, +10.8)
    pt_ee_start = pt_along(p_ne, u_e, n_e, s_e_split, -10.8)
    pt_ee_end = pt_along(p_ne, u_e, n_e, s_ee_end, -10.8)

    # 2. Delete N2_CROSS_W, N2_CROSS_E, NORTH_OUTLET_M, NORTH_OUTLET_W, NORTH_OUTLET_E & junction N2_M
    for del_rid in ("N2_CROSS_W", "N2_CROSS_E", "NORTH_OUTLET_M", "NORTH_OUTLET_W", "NORTH_OUTLET_E"):
        roads_s.pop(del_rid, None)
    juncs_s.pop("N2_M", None)

    # Update N2_W and N2_E to virtual lane_split junctions (no_world_node = True)
    juncs_s["N2_W"] = {
        "position": pt_w_split,
        "kind": "lane_split",
        "no_world_node": True,
        "trim": 0,
        "ports": [],
        "world_id": "J_N2_W",
        "polygon": [],
    }
    juncs_s["N2_E"] = {
        "position": pt_e_split,
        "kind": "lane_split",
        "no_world_node": True,
        "trim": 0,
        "ports": [],
        "world_id": "J_N2_E",
        "polygon": [],
    }

    # 3. Update NORTH_FAN_W, NORTH_FAN_M, NORTH_FAN_E
    roads_s["NORTH_FAN_W"].update({
        "from": "NORTH_WEST",
        "to": "N2_W",
        "points": [pt_w_start, pt_w_split],
        "elevations": [0.048, 0.048],
        "full_points": [p_nw, pt_w_start, pt_w_split, pt_w_split],
        "number_of_lanes": 8,
        "forward_lanes": 4,
        "width": 28.8,
    })
    roads_s["NORTH_FAN_M"].update({
        "from": "NORTH_MIDDLE",
        "to": "O_N_M",
        "points": [pt_m_start, pt_m_end],
        "elevations": [0.048, 0.048],
        "full_points": [p_nm, pt_m_start, pt_m_end, p_onm],
        "number_of_lanes": 4,
        "forward_lanes": 2,
        "width": 14.4,
    })
    roads_s["NORTH_FAN_E"].update({
        "from": "NORTH_EAST",
        "to": "N2_E",
        "points": [pt_e_start, pt_e_split],
        "elevations": [0.048, 0.048],
        "full_points": [p_ne, pt_e_start, pt_e_split, pt_e_split],
        "number_of_lanes": 8,
        "forward_lanes": 4,
        "width": 28.8,
    })

    # 4. Trim OUTER_RING_NW, NORTH_WINDING_W, NORTH_WINDING_E, OUTER_RING_NE at O_N_W, O_N_M, O_N_E
    roads_s["OUTER_RING_NW"]["points"] = roads_s["OUTER_RING_NW"]["points"][:-3]
    roads_s["OUTER_RING_NW"]["elevations"] = roads_s["OUTER_RING_NW"]["elevations"][:-3]
    roads_s["OUTER_RING_NW"]["full_points"] = [juncs_s["O_W_N"]["position"]] + roads_s["OUTER_RING_NW"]["points"] + [p_onw]

    roads_s["NORTH_WINDING_W"]["points"] = roads_s["NORTH_WINDING_W"]["points"][2:-1]
    roads_s["NORTH_WINDING_W"]["elevations"] = roads_s["NORTH_WINDING_W"]["elevations"][2:-1]
    roads_s["NORTH_WINDING_W"]["full_points"] = [p_onw] + roads_s["NORTH_WINDING_W"]["points"] + [p_onm]

    roads_s["NORTH_WINDING_E"]["points"] = roads_s["NORTH_WINDING_E"]["points"][1:-4]
    roads_s["NORTH_WINDING_E"]["elevations"] = roads_s["NORTH_WINDING_E"]["elevations"][1:-4]
    roads_s["NORTH_WINDING_E"]["full_points"] = [p_onm] + roads_s["NORTH_WINDING_E"]["points"] + [p_one]

    roads_s["OUTER_RING_NE"]["points"] = roads_s["OUTER_RING_NE"]["points"][3:]
    roads_s["OUTER_RING_NE"]["elevations"] = roads_s["OUTER_RING_NE"]["elevations"][3:]
    roads_s["OUTER_RING_NE"]["full_points"] = [p_one] + roads_s["OUTER_RING_NE"]["points"] + [juncs_s["O_E_M"]["position"]]

    # 5. Add 4 outer 2-lane ground roads collinear with NORTH_FAN_W and NORTH_FAN_E
    outlet_specs = [
        ("NORTH_OUTLET_W_W", "N2_W", "O_N_W", [pt_ww_start, pt_ww_end], 0),
        ("NORTH_OUTLET_W_E", "N2_W", "O_N_W", [pt_we_start, pt_we_end], 2),
        ("NORTH_OUTLET_E_W", "N2_E", "O_N_E", [pt_ew_start, pt_ew_end], 0),
        ("NORTH_OUTLET_E_E", "N2_E", "O_N_E", [pt_ee_start, pt_ee_end], 2),
    ]
    for rid, fj, tj, pts, fwd in outlet_specs:
        roads_s[rid] = {
            "id": rid,
            "name": f"Road-{rid}",
            "from": fj,
            "to": tj,
            "points": pts,
            "elevations": [0.048, 0.048],
            "full_points": [juncs_s[fj]["position"]] + pts + [juncs_s[tj]["position"]],
            "width": 7.2,
            "number_of_lanes": 2,
            "forward_lanes": fwd,
            "training": False,
        }

    # Update map_s['roads'] order
    new_roads_list_s = []
    for r in map_s["roads"]:
        rid = r["id"]
        if rid == "NORTH_OUTLET_W":
            new_roads_list_s.append(roads_s["NORTH_OUTLET_W_W"])
            new_roads_list_s.append(roads_s["NORTH_OUTLET_W_E"])
        elif rid == "NORTH_OUTLET_E":
            new_roads_list_s.append(roads_s["NORTH_OUTLET_E_W"])
            new_roads_list_s.append(roads_s["NORTH_OUTLET_E_E"])
        elif rid in roads_s:
            new_roads_list_s.append(roads_s[rid])
    map_s["roads"] = new_roads_list_s

    # Update expansion['new_road_ids']
    removed_ids = {"N2_CROSS_W", "N2_CROSS_E", "NORTH_OUTLET_M", "NORTH_OUTLET_W", "NORTH_OUTLET_E",
                   "NORTH_OUTLET_W_W", "NORTH_OUTLET_W_E", "NORTH_OUTLET_E_W", "NORTH_OUTLET_E_E"}
    exp_ids = [eid for eid in map_s["expansion"]["new_road_ids"] if eid not in removed_ids]
    exp_ids.extend(["NORTH_OUTLET_W_W", "NORTH_OUTLET_W_E", "NORTH_OUTLET_E_W", "NORTH_OUTLET_E_E"])
    map_s["expansion"]["new_road_ids"] = exp_ids

    # Update junctions ports & polygons for modified South junctions
    mod_juncs_s = ("NORTH_WEST", "NORTH_MIDDLE", "NORTH_EAST", "N2_W", "N2_E", "O_N_W", "O_N_M", "O_N_E")
    for jid in mod_juncs_s:
        juncs_s[jid]["ports"] = []
    for r in map_s["roads"]:
        eid = r["id"]
        pts = r["points"]
        for nid, p, q in [(r["from"], pts[0], pts[1]), (r["to"], pts[-1], pts[-2])]:
            if nid in mod_juncs_s:
                juncs_s[nid]["ports"].append({
                    "edge": eid,
                    "position": p,
                    "outward": list(g.direction(p, q)),
                    "width": r["width"],
                })
    for jid in mod_juncs_s:
        if juncs_s[jid]["kind"] == "lane_split":
            juncs_s[jid]["polygon"] = []
        else:
            verts = [pt for port in juncs_s[jid]["ports"] for pt in mouth(port["position"], port["outward"], port["width"])]
            juncs_s[jid]["polygon"] = [[round(x, 6), round(y, 6)] for x, y in g.hull(verts)]

    # Update medians in map_s
    medians_s = [m for m in map_s["medians"] if m["road_id"] not in ("NORTH_FAN_W", "NORTH_FAN_E")]
    for rid in ("NORTH_FAN_W", "NORTH_FAN_E"):
        r = roads_s[rid]
        medians_s.append({
            "road_id": rid,
            "width": 0.6,
            "height": 0.6,
            "sections": [[[*p, 0.048] for p in r["points"]]],
        })
    map_s["medians"] = medians_s

    # Rebuild South lane_graph
    lane_graph_s = {}
    for r in map_s["roads"]:
        generate_lanes_for_road_6x(r, lane_graph_s)
    ring = map_s["roundabout"]
    for index in range(1, ring["ring_lanes"] + 1):
        for i, nid in enumerate(["R_E", "R_N", "R_W", "R_S"]):
            nxt = ["R_E", "R_N", "R_W", "R_S"][(i + 1) % 4]
            r_rad = ring["inner_radius"] + (ring["outer_radius"] - ring["inner_radius"]) / ring["ring_lanes"] * (index - 0.5)
            lane_graph_s[f"RING_{nid}_{index}"] = {
                "from": nid,
                "to": nxt,
                "points": [[round(x, 6), round(y, 6)] for x, y in g.arc(ring["center"], r_rad, i * math.pi / 2, (i + 1) * math.pi / 2)],
                "lane_index": index,
                "successors": [],
                "speed_limit_m_s": 0.25,
            }

    # Trim inner bridge lanes (F1, F2, B1, B2) at N2_W and N2_E back by 6.2m so connectors stay > 1.116m from CentreMedian
    for r_fan in ("NORTH_FAN_W", "NORTH_FAN_E"):
        for idx in (1, 2):
            f_lane = lane_graph_s[f"{r_fan}_F{idx}_exit"]
            f_pts = f_lane["points"]
            f_lane["points"] = [[round(x, 6), round(y, 6)] for x, y in g.cut(f_pts, 0, 6.2)]
            b_lane = lane_graph_s[f"{r_fan}_B{idx}_entry"]
            b_pts = b_lane["points"]
            b_lane["points"] = [[round(x, 6), round(y, 6)] for x, y in g.cut(b_pts, 6.2, 0)]

    def matching_lane(index, count, other, other_count):
        return max((index - 1) / count, (other - 1) / other_count) < min(index / count, other / other_count) - 1e-9

    paired_outlets = {
        ("NORTH_OUTLET_W_W", "NORTH_OUTLET_W_E"),
        ("NORTH_OUTLET_W_E", "NORTH_OUTLET_W_W"),
        ("NORTH_OUTLET_E_W", "NORTH_OUTLET_E_E"),
        ("NORTH_OUTLET_E_E", "NORTH_OUTLET_E_W"),
    }
    original_lanes = list(lane_graph_s.items())
    for lid, lane in original_lanes:
        nid = lane["to"]
        for other, target in original_lanes:
            if target["from"] != nid:
                continue
            if not nid.startswith("CHANGE_") and lane.get("road_id") and lane.get("road_id") == target.get("road_id"):
                continue
            if (lane.get("road_id"), target.get("road_id")) in paired_outlets:
                continue
            if juncs_s.get(nid, {}).get("kind") == "lane_split":
                if math.dist(lane["points"][-1], target["points"][0]) < 0.05:
                    lane["successors"].append(other)
                elif nid in ("N2_W", "N2_E"):
                    if lane["lane_index"] in (1, 2) and target["lane_index"] == 1:
                        a, d = lane["points"][-1], target["points"][0]
                        u = g.direction(lane["points"][-2], a)
                        v = g.direction(d, target["points"][1])
                        reach = 4.0
                        cid = f"{lid}__{other}"
                        lane_graph_s[cid] = {
                            "from": nid,
                            "to": nid,
                            "points": [[round(x, 6), round(y, 6)] for x, y in g.bezier(a, g.add(a, g.mul(u, reach)), g.add(d, g.mul(v, -reach)), d, 12)],
                            "successors": [other],
                            "speed_limit_m_s": 0.25,
                            "junction": nid,
                        }
                        lane["successors"].append(cid)
                    elif lane["lane_index"] == 1 and lane["direction_lanes"] == 2 and target["lane_index"] in (1, 2) and target["direction_lanes"] == 4:
                        a, d = lane["points"][-1], target["points"][0]
                        u = g.direction(lane["points"][-2], a)
                        v = g.direction(d, target["points"][1])
                        reach = 4.0
                        cid = f"{lid}__{other}"
                        lane_graph_s[cid] = {
                            "from": nid,
                            "to": nid,
                            "points": [[round(x, 6), round(y, 6)] for x, y in g.bezier(a, g.add(a, g.mul(u, reach)), g.add(d, g.mul(v, -reach)), d, 12)],
                            "successors": [other],
                            "speed_limit_m_s": 0.25,
                            "junction": nid,
                        }
                        lane["successors"].append(cid)
                continue
            if nid.startswith("R_") and not (lid.startswith("RING_") or other.startswith("RING_")):
                continue
            if nid.startswith("R_") and lane.get("lane_index") != target.get("lane_index"):
                continue
            turn = 0
            if not nid.startswith(("CHANGE_", "R_")) and "lane_index" in lane and "lane_index" in target:
                incoming = g.direction(lane["points"][-2], lane["points"][-1])
                outgoing = g.direction(target["points"][0], target["points"][1])
                turn = math.atan2(incoming[0] * outgoing[1] - incoming[1] * outgoing[0], incoming[0] * outgoing[0] + incoming[1] * outgoing[1])
                if abs(turn) < 0.65:
                    if not matching_lane(lane["lane_index"], lane["direction_lanes"], target["lane_index"], target["direction_lanes"]):
                        continue
                elif turn > 0:
                    count, other_count = math.ceil(lane["direction_lanes"] / 2), math.ceil(target["direction_lanes"] / 2)
                    if lane["lane_index"] > count or target["lane_index"] > other_count or not matching_lane(lane["lane_index"], count, target["lane_index"], other_count):
                        continue
                else:
                    index = lane["lane_index"] - lane["direction_lanes"] // 2
                    other_index = target["lane_index"] - target["direction_lanes"] // 2
                    if index <= 0 or other_index <= 0 or not matching_lane(index, math.ceil(lane["direction_lanes"] / 2), other_index, math.ceil(target["direction_lanes"] / 2)):
                        continue
            a, d = lane["points"][-1], target["points"][0]
            if math.dist(a, d) < 0.01:
                lane["successors"].append(other)
                continue
            u = g.direction(lane["points"][-2], a)
            v = g.direction(d, target["points"][1])
            reach = math.dist(a, d) * (0.05 if 0.15 < abs(turn) < 0.65 else 0.6 if nid == "SIGNAL_CROSS" else 0.45)
            cid = f"{lid}__{other}"
            lane_graph_s[cid] = {
                "from": nid,
                "to": nid,
                "points": [[round(x, 6), round(y, 6)] for x, y in g.bezier(a, g.add(a, g.mul(u, reach)), g.add(d, g.mul(v, -reach)), d, 16)],
                "successors": [other],
                "speed_limit_m_s": 0.25,
                "junction": nid,
            }
            if nid.startswith("CHANGE_"):
                lane_graph_s[cid]["road_id"] = lane["road_id"]
            lane["successors"].append(cid)

    for lid, lane in lane_graph_s.items():
        support = [roads_s[lane["road_id"]]] if lane.get("road_id") in roads_s else []
        lane["elevations"] = [round(road_height(p, support) if support else 0.048, 6) for p in lane["points"]]
    map_s["lanes"] = lane_graph_s

    # Update South WBT nodes in prefix_str
    nodes_s = read_nodes(prefix_str)

    def make_8lane_straight_wbt(r: dict) -> str:
        p0, p1 = r["points"]
        angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        length = math.dist(p0, p1)
        fj_w = juncs_s[r["from"]]["world_id"]
        tj_w = juncs_s[r["to"]]["world_id"]
        tex = "../../../../../../worlds/textures/stop_line.png"
        return f"""StraightRoadSegment {{
  translation {vec((*p0, 0.048))}
  rotation 0 0 1 {fmt(angle)}
  name "{r['name']}"
  id "{r['id']}"
  startJunction "{fj_w}"
  endJunction "{tj_w}"
  width 28.8
  numberOfLanes 8
  numberOfForwardLanes 4
  lines [
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
      color 0.85 0.75 0.3
      type "none"
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
  ]
  roadBorderHeight 0.21
  startingRoadBorderWidth 0.84
  endingRoadBorderWidth 0.84
  length {fmt(length)}
  startLine [
    ""
    ""
    ""
    ""
    "{tex}"
    "{tex}"
    "{tex}"
    "{tex}"
  ]
  endLine [
  ]
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}"""

    def make_north_fan_m_wbt(r: dict) -> str:
        p0, p1 = r["points"]
        angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        length = math.dist(p0, p1)
        fj_w = juncs_s[r["from"]]["world_id"]
        tj_w = juncs_s[r["to"]]["world_id"]
        tex_stop = "../../../../../../worlds/textures/stop_line.png"
        tex_cross = "../../../../../../worlds/textures/crossing.png"
        tex_stop_cross = "../../../../../../worlds/textures/stop_crossing.png"
        return f"""StraightRoadSegment {{
  translation {vec((*p0, 0.048))}
  rotation 0 0 1 {fmt(angle)}
  name "{r['name']}"
  id "{r['id']}"
  startJunction "{fj_w}"
  endJunction "{tj_w}"
  width 14.4
  numberOfLanes 4
  numberOfForwardLanes 2
  lines [
    RoadLine {{
    }}
    RoadLine {{
      color 0.85 0.75 0.3
      type "double"
    }}
    RoadLine {{
    }}
  ]
  roadBorderHeight 0.21
  startingRoadBorderWidth 0.84
  endingRoadBorderWidth 0.84
  length {fmt(length)}
  startLine [
    "{tex_cross}"
    "{tex_cross}"
    "{tex_stop_cross}"
    "{tex_stop_cross}"
  ]
  endLine [
    "{tex_stop}"
    "{tex_stop}"
    ""
    ""
  ]
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}"""

    def make_2lane_outlet_wbt(r: dict) -> str:
        p0, p1 = r["points"]
        angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        length = math.dist(p0, p1)
        fj_w = juncs_s[r["from"]]["world_id"]
        tj_w = juncs_s[r["to"]]["world_id"]
        fwd = r["forward_lanes"]
        tex = "../../../../../../worlds/textures/stop_line.png"
        if fwd == 0:
            border_off = "rightBorder FALSE"
            bo_on = "leftBorderBoundingObject TRUE"
            el = '[\n    ""\n    ""\n  ]'
        else:
            border_off = "leftBorder FALSE"
            bo_on = "rightBorderBoundingObject TRUE"
            el = f'[\n    "{tex}"\n    "{tex}"\n  ]'
        return f"""StraightRoadSegment {{
  translation {vec((*p0, 0.048))}
  rotation 0 0 1 {fmt(angle)}
  name "{r['name']}"
  id "{r['id']}"
  startJunction "{fj_w}"
  endJunction "{tj_w}"
  width 7.2
  numberOfForwardLanes {fwd}
  lines [
    RoadLine {{
      width 0.15
    }}
  ]
  roadBorderHeight 0.21
  startingRoadBorderWidth 0.84
  endingRoadBorderWidth 0.84
  {border_off}
  length {fmt(length)}
  startLine [
  ]
  endLine {el}
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  {bo_on}
}}"""

    for n in nodes_s:
        eid = n.get("id")
        name = n.get("name", "")
        desc = n.get("description", "")
        if eid in ("NORTH_FAN_W", "NORTH_FAN_E"):
            n.text = make_8lane_straight_wbt(roads_s[eid])
        elif eid == "NORTH_FAN_M":
            n.text = make_north_fan_m_wbt(roads_s["NORTH_FAN_M"])
        elif eid in ("N2_CROSS_W", "N2_CROSS_E", "NORTH_OUTLET_M", "J_N2_W", "J_N2_M", "J_N2_E"):
            n.text = ""
        elif eid == "NORTH_OUTLET_W":
            n.text = make_2lane_outlet_wbt(roads_s["NORTH_OUTLET_W_W"]) + "\n\n" + make_2lane_outlet_wbt(roads_s["NORTH_OUTLET_W_E"])
        elif eid == "NORTH_OUTLET_E":
            n.text = make_2lane_outlet_wbt(roads_s["NORTH_OUTLET_E_W"]) + "\n\n" + make_2lane_outlet_wbt(roads_s["NORTH_OUTLET_E_E"])
        elif eid in ("OUTER_RING_NW", "NORTH_WINDING_W", "NORTH_WINDING_E", "OUTER_RING_NE"):
            r = roads_s[eid]
            wps_str = "\n    ".join(f"{fmt(p[0])} {fmt(p[1])} {fmt(z)}" for p, z in zip(r["points"], r["elevations"]))
            n.set("wayPoints", f"[\n    {wps_str}\n  ]")
        elif eid in ("J_NORTH_WEST", "J_NORTH_MIDDLE", "J_NORTH_EAST", "J_O_N_W", "J_O_N_M", "J_O_N_E"):
            jkey = eid[2:]
            j = juncs_s[jkey]
            shape_str = "\n    ".join(vec((-y, -x, 0)) for x, y in j["polygon"])
            conn_str = "\n    ".join(f'"{p["edge"]}"' for p in j["ports"])
            n.set("shape", f"[\n    {shape_str}\n  ]")
            n.set("connectedRoadIDs", f"[\n    {conn_str}\n  ]")
        elif name.startswith("JunctionCurbs-"):
            jkey = name[len("JunctionCurbs-"):]
            if jkey in ("N2_W", "N2_M", "N2_E"):
                n.text = ""
            elif jkey in ("NORTH_WEST", "NORTH_MIDDLE", "NORTH_EAST", "O_N_W", "O_N_M", "O_N_E"):
                n.text = build_south_curb_solid(jkey, juncs_s[jkey])
        elif name in ("CentreMedian-NORTH_FAN_W", "CentreMedian-NORTH_FAN_E"):
            n.text = ""
        elif desc == "SIGN_NORTH_MIDDLE":
            n.set("translation", "27.090472 154.423332 0")
        elif desc == "SIGN_NORTH_EAST":
            n.set("translation", "118.328311 155.120776 0")

    # Append the 2 new South median solids before # BEGIN NORTH_DISTRICT
    new_south_medians_wbt = []
    for rid in ("NORTH_FAN_W", "NORTH_FAN_E"):
        med = next(m for m in map_s["medians"] if m["road_id"] == rid)
        new_south_medians_wbt.append(build_south_median_solid(med))

    new_prefix_str = prefix_str
    for n in reversed(nodes_s):
        new_prefix_str = new_prefix_str[:n.start] + n.text + new_prefix_str[n.end:]
    new_prefix_str = re.sub(r"\n{3,}", "\n\n", new_prefix_str.rstrip()) + "\n\n" + "\n\n".join(new_south_medians_wbt) + "\n\n"

    # =========================================================================
    # PART 2: UPDATE NORTH DISTRICT & ADD 2 RIVER-CROSSING BRIDGES
    # =========================================================================
    pts_bridge_w = make_bridge_waypoints(
        p_start=pt_w_split,
        u_start=u_w,
        s_ramp_s=32.0,
        p_north_ramp_start=[-105.0, 454.0],
        p_end=[-105.0, 486.0],
        n_south=8, n_mid=50, n_north=8,
    )
    pts_bridge_e = make_bridge_waypoints(
        p_start=pt_e_split,
        u_start=u_e,
        s_ramp_s=32.0,
        p_north_ramp_start=[105.0, 448.0],
        p_end=[105.0, 480.0],
        n_south=8, n_mid=50, n_north=8,
    )

    roads_n = {r["id"]: r for r in map_n["roads"]}
    juncs_n = map_n["junctions"]

    # 1. Update existing North District roads
    roads_n["ND_MAIN_S"]["from"] = "ND_SPLIT_RIVER_E"
    roads_n["ND_MAIN_S"]["to"] = "ND_SPLIT_S"
    roads_n["ND_MAIN_S"]["points"] = [[105.0, 480.0], [105.0, 628.5]]
    roads_n["ND_MAIN_S"]["elevations"] = [0.048, 0.048]

    roads_n["ND_ROUND_SOUTH_LINK"]["from"] = "ND_J_WS_T"
    roads_n["ND_ROUND_SOUTH_LINK"]["to"] = "ND_SPLIT_RIVER_W"
    roads_n["ND_ROUND_SOUTH_LINK"]["number_of_lanes"] = 8
    roads_n["ND_ROUND_SOUTH_LINK"]["forward_lanes"] = 4
    roads_n["ND_ROUND_SOUTH_LINK"]["width"] = 28.8
    roads_n["ND_ROUND_SOUTH_LINK"]["points"] = [[-105.0, 562.2], [-105.0, 486.0]]
    roads_n["ND_ROUND_SOUTH_LINK"]["elevations"] = [0.048, 0.048]

    roads_n["ND_LOCAL_W"]["points"][-1][0] = -120.6

    sw_pts = [p for p in roads_n["ND_OUTER_SW"]["points"] if p[0] < -121.0]
    sw_pts.append([-120.6, 434.9412])
    roads_n["ND_OUTER_SW"]["points"] = sw_pts
    roads_n["ND_OUTER_SW"]["elevations"] = [0.048] * len(sw_pts)

    shw_pts = [[-89.4, 434.9412]] + [p for p in roads_n["ND_SHORE_W"]["points"] if p[0] > -88.5]
    roads_n["ND_SHORE_W"]["points"] = shw_pts
    roads_n["ND_SHORE_W"]["elevations"] = [0.048] * len(shw_pts)

    # 2. Add 6 new roads to map_n
    new_nd_roads = [
        {
            "id": "ND_RIVER_BRIDGE_E",
            "name": "Road-ND_RIVER_BRIDGE_E",
            "from": "ND_SPLIT_RIVER_ES",
            "to": "ND_SPLIT_RIVER_E",
            "number_of_lanes": 4,
            "forward_lanes": 2,
            "width": 14.4,
            "points": [[p[0], p[1]] for p in pts_bridge_e],
            "elevations": [p[2] for p in pts_bridge_e],
        },
        {
            "id": "ND_RIVER_E_GROUND_W",
            "name": "Road-ND_RIVER_E_GROUND_W",
            "from": "ND_J_SC_T",
            "to": "ND_SPLIT_RIVER_E",
            "number_of_lanes": 2,
            "forward_lanes": 0,
            "width": 7.2,
            "points": [[94.2, 439.9062], [94.2, 480.0]],
            "elevations": [0.048, 0.048],
        },
        {
            "id": "ND_RIVER_E_GROUND_E",
            "name": "Road-ND_RIVER_E_GROUND_E",
            "from": "ND_J_SC_T",
            "to": "ND_SPLIT_RIVER_E",
            "number_of_lanes": 2,
            "forward_lanes": 2,
            "width": 7.2,
            "points": [[115.8, 439.9062], [115.8, 480.0]],
            "elevations": [0.048, 0.048],
        },
        {
            "id": "ND_RIVER_BRIDGE_W",
            "name": "Road-ND_RIVER_BRIDGE_W",
            "from": "ND_SPLIT_RIVER_WS",
            "to": "ND_SPLIT_RIVER_W",
            "number_of_lanes": 4,
            "forward_lanes": 2,
            "width": 14.4,
            "points": [[p[0], p[1]] for p in pts_bridge_w],
            "elevations": [p[2] for p in pts_bridge_w],
        },
        {
            "id": "ND_RIVER_W_GROUND_W",
            "name": "Road-ND_RIVER_W_GROUND_W",
            "from": "ND_J_SW_T",
            "to": "ND_SPLIT_RIVER_W",
            "number_of_lanes": 2,
            "forward_lanes": 0,
            "width": 7.2,
            "points": [[-115.8, 445.7412], [-115.8, 486.0]],
            "elevations": [0.048, 0.048],
        },
        {
            "id": "ND_RIVER_W_GROUND_E",
            "name": "Road-ND_RIVER_W_GROUND_E",
            "from": "ND_J_SW_T",
            "to": "ND_SPLIT_RIVER_W",
            "number_of_lanes": 2,
            "forward_lanes": 2,
            "width": 7.2,
            "points": [[-94.2, 445.7412], [-94.2, 486.0]],
            "elevations": [0.048, 0.048],
        },
    ]
    for r in new_nd_roads:
        roads_n[r["id"]] = r
    map_n["roads"] = [roads_n[rid] for rid in [r["id"] for r in map_n["roads"] if r["id"] not in {nr["id"] for nr in new_nd_roads}]] + new_nd_roads

    # 3. Update & Add North District junctions
    juncs_n["ND_J_WS_T"] = {
        "world_id": "ND_J_WS_T",
        "position": [-105.0, 573.0],
        "kind": "T",
        "shape": [[-112.2, 583.8], [-97.8, 583.8], [-90.6, 562.2], [-119.4, 562.2], [-120.6, 569.4], [-120.6, 576.6]],
        "ports": [
            {"edge": "ND_ROUND_S", "position": [-105.0, 583.8], "outward": [0.0, 1.0], "width": 14.4},
            {"edge": "ND_ROUND_SOUTH_LINK", "position": [-105.0, 562.2], "outward": [0.0, -1.0], "width": 28.8},
            {"edge": "ND_LOCAL_W", "position": [-120.6, 573.0], "outward": [-1.0, 0.0], "width": 7.2},
        ],
    }
    juncs_n["ND_J_SW_T"] = {
        "world_id": "ND_J_SW_T",
        "position": [-105.0, 434.9412],
        "kind": "T",
        "shape": [[-120.6, 427.7412], [-120.6, 442.1412], [-119.4, 445.7412], [-90.6, 445.7412], [-89.4, 442.1412], [-89.4, 427.7412]],
        "ports": [
            {"edge": "ND_OUTER_SW", "position": [-120.6, 434.9412], "outward": [-1.0, 0.0], "width": 14.4},
            {"edge": "ND_SHORE_W", "position": [-89.4, 434.9412], "outward": [1.0, 0.0], "width": 14.4},
            {"edge": "ND_RIVER_W_GROUND_W", "position": [-115.8, 445.7412], "outward": [0.0, 1.0], "width": 7.2},
            {"edge": "ND_RIVER_W_GROUND_E", "position": [-94.2, 445.7412], "outward": [0.0, 1.0], "width": 7.2},
        ],
    }
    juncs_n["ND_J_SC_T"] = {
        "world_id": "ND_J_SC_T",
        "position": [105.0, 424.3062],
        "kind": "T",
        "shape": [[89.4, 417.1062], [89.4, 431.5062], [90.6, 439.9062], [119.4, 439.9062], [120.6, 431.5062], [120.6, 417.1062]],
        "ports": [
            {"edge": "ND_SHORE_M", "position": [89.4, 424.3062], "outward": [-1.0, 0.0], "width": 14.4},
            {"edge": "ND_SHORE_E", "position": [120.6, 424.3062], "outward": [1.0, 0.0], "width": 14.4},
            {"edge": "ND_RIVER_E_GROUND_W", "position": [94.2, 439.9062], "outward": [0.0, 1.0], "width": 7.2},
            {"edge": "ND_RIVER_E_GROUND_E", "position": [115.8, 439.9062], "outward": [0.0, 1.0], "width": 7.2},
        ],
    }
    juncs_n["ND_SPLIT_RIVER_E"] = {
        "world_id": "ND_SPLIT_RIVER_E",
        "position": [105.0, 480.0],
        "kind": "lane_split",
        "shape": [],
        "ports": [
            {"edge": "ND_MAIN_S", "position": [105.0, 480.0], "outward": [0.0, 1.0], "width": 28.8},
            {"edge": "ND_RIVER_BRIDGE_E", "position": [105.0, 480.0], "outward": [0.0, -1.0], "width": 14.4},
            {"edge": "ND_RIVER_E_GROUND_W", "position": [94.2, 480.0], "outward": [0.0, -1.0], "width": 7.2},
            {"edge": "ND_RIVER_E_GROUND_E", "position": [115.8, 480.0], "outward": [0.0, -1.0], "width": 7.2},
        ],
    }
    juncs_n["ND_SPLIT_RIVER_ES"] = {
        "world_id": "ND_SPLIT_RIVER_ES",
        "position": pt_e_split,
        "kind": "lane_split",
        "shape": [],
        "ports": [
            {"edge": "ND_RIVER_BRIDGE_E", "position": pt_e_split, "outward": [round(u_e[0], 6), round(u_e[1], 6)], "width": 14.4},
        ],
    }
    juncs_n["ND_SPLIT_RIVER_W"] = {
        "world_id": "ND_SPLIT_RIVER_W",
        "position": [-105.0, 486.0],
        "kind": "lane_split",
        "shape": [],
        "ports": [
            {"edge": "ND_ROUND_SOUTH_LINK", "position": [-105.0, 486.0], "outward": [0.0, 1.0], "width": 28.8},
            {"edge": "ND_RIVER_BRIDGE_W", "position": [-105.0, 486.0], "outward": [0.0, -1.0], "width": 14.4},
            {"edge": "ND_RIVER_W_GROUND_W", "position": [-115.8, 486.0], "outward": [0.0, -1.0], "width": 7.2},
            {"edge": "ND_RIVER_W_GROUND_E", "position": [-94.2, 486.0], "outward": [0.0, -1.0], "width": 7.2},
        ],
    }
    juncs_n["ND_SPLIT_RIVER_WS"] = {
        "world_id": "ND_SPLIT_RIVER_WS",
        "position": pt_w_split,
        "kind": "lane_split",
        "shape": [],
        "ports": [
            {"edge": "ND_RIVER_BRIDGE_W", "position": pt_w_split, "outward": [round(u_w[0], 6), round(u_w[1], 6)], "width": 14.4},
        ],
    }

    map_n["river_bridges"] = {
        "east_bridge": {
            "id": "ND_RIVER_BRIDGE_E",
            "lanes": 4,
            "width_m": 14.4,
            "parapet_height_m": 1.5,
            "median_type": "columns",
            "column_count": 33,
            "max_elevation_m": 7.5,
            "max_slope": 0.2188,
            "south_touchdown": pt_e_split,
            "north_touchdown": [105.0, 480.0],
        },
        "west_bridge": {
            "id": "ND_RIVER_BRIDGE_W",
            "lanes": 4,
            "width_m": 14.4,
            "parapet_height_m": 1.5,
            "median_type": "columns",
            "column_count": 33,
            "max_elevation_m": 7.5,
            "max_slope": 0.2188,
            "south_touchdown": pt_w_split,
            "north_touchdown": [-105.0, 486.0],
        },
    }

    map_n["lanes"] = generate_north_lanes(map_n["roads"])

    # 4. Update & Add WBT nodes in nd_str
    nodes_n = read_nodes(nd_str)
    added_nd_ids = {"ND_SPLIT_RIVER_E", "ND_SPLIT_RIVER_ES", "ND_SPLIT_RIVER_W", "ND_SPLIT_RIVER_WS",
                    "ND_RIVER_BRIDGE_E", "ND_RIVER_E_GROUND_W", "ND_RIVER_E_GROUND_E",
                    "ND_RIVER_BRIDGE_W", "ND_RIVER_W_GROUND_W", "ND_RIVER_W_GROUND_E"}
    added_nd_names = {"ND_Median-ROUND_SOUTH_LINK", "ND_Median-RIVER_BRIDGE_E", "ND_Median-RIVER_BRIDGE_W",
                      "ND_MedianColumns-RIVER_BRIDGE_E", "ND_MedianColumns-RIVER_BRIDGE_W",
                      "ND_RiverBridgeE-Deck", "ND_RiverBridgeE-RampSupport-South", "ND_RiverBridgeE-RampSupport-North",
                      "ND_RiverBridgeE-Parapet-West", "ND_RiverBridgeE-Parapet-East", "ND_RiverBridgeE-Piers",
                      "ND_RiverBridgeW-Deck", "ND_RiverBridgeW-RampSupport-South", "ND_RiverBridgeW-RampSupport-North",
                      "ND_RiverBridgeW-Parapet-West", "ND_RiverBridgeW-Parapet-East", "ND_RiverBridgeW-Piers"}

    def nd_crossroad_wbt(jid: str, name: str, shape_pts: list, road_ids: list) -> str:
        if shape_pts:
            signed_area = sum(shape_pts[i][0] * shape_pts[(i + 1) % len(shape_pts)][1] - shape_pts[(i + 1) % len(shape_pts)][0] * shape_pts[i][1] for i in range(len(shape_pts)))
            ordered_pts = list(shape_pts) if signed_area > 0 else list(reversed(shape_pts))
            shape_str = "[\n    " + "\n    ".join(f"{fmt(-p[1])} {fmt(-p[0])} 0" for p in ordered_pts) + "\n  ]"
        else:
            shape_str = "[]"
        roads_str = "\n    ".join(f'"{rid}"' for rid in road_ids)
        return f"""Crossroad {{
  translation 0 0 0.048
  name "{name}"
  id "{jid}"
  shape {shape_str}
  connectedRoadIDs [
    {roads_str}
  ]
  bottom TRUE
  appearance {ASPHALT}
}}"""

    tex_stop = "../../../../../../worlds/textures/stop_line.png"
    tex_stop_cross = "../../../../../../worlds/textures/stop_crossing.png"
    tex_cross = "../../../../../../worlds/textures/crossing.png"

    for n in nodes_n:
        eid = n.get("id")
        name = n.get("name", "")
        if eid in added_nd_ids or name in added_nd_names:
            n.text = ""
            continue
        if eid == "ND_J_WS_T":
            n.text = nd_crossroad_wbt("ND_J_WS_T", "ND_Junction-WS_T", juncs_n["ND_J_WS_T"]["shape"], ["ND_ROUND_S", "ND_ROUND_SOUTH_LINK", "ND_LOCAL_W"])
        elif eid == "ND_J_SW_T":
            n.text = nd_crossroad_wbt("ND_J_SW_T", "ND_Junction-SW_T", juncs_n["ND_J_SW_T"]["shape"], ["ND_OUTER_SW", "ND_SHORE_W", "ND_RIVER_W_GROUND_W", "ND_RIVER_W_GROUND_E"])
        elif eid == "ND_J_SC_T":
            n.text = nd_crossroad_wbt("ND_J_SC_T", "ND_Junction-SC_T", juncs_n["ND_J_SC_T"]["shape"], ["ND_SHORE_M", "ND_SHORE_E", "ND_RIVER_E_GROUND_W", "ND_RIVER_E_GROUND_E"])
        elif eid == "ND_MAIN_S":
            n.text = f"""Road {{
  name "Road-ND_MAIN_S"
  id "ND_MAIN_S"
  startJunction "ND_SPLIT_RIVER_E"
  endJunction "ND_SPLIT_S"
  width 28.8
  numberOfLanes 8
  numberOfForwardLanes 4
  lines [
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
      color 0.85 0.75 0.3
      type "none"
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
  ]
  roadBorderHeight 0.21
  roadBorderWidth [
    0.14
  ]
  wayPoints [
    105 480 0.048
    105 628.5 0.048
  ]
  startLine [
    ""
    ""
    ""
    ""
    ""
    ""
    ""
    ""
  ]
  endLine [
    ""
    ""
    ""
    ""
    ""
    ""
    ""
    ""
  ]
  splineSubdivision -1
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}"""
        elif eid == "ND_ROUND_SOUTH_LINK":
            n.text = f"""Road {{
  name "Road-ND_ROUND_SOUTH_LINK"
  id "ND_ROUND_SOUTH_LINK"
  startJunction "ND_J_WS_T"
  endJunction "ND_SPLIT_RIVER_W"
  width 28.8
  numberOfLanes 8
  numberOfForwardLanes 4
  lines [
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
      color 0.85 0.75 0.3
      type "none"
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
    RoadLine {{
    }}
  ]
  roadBorderHeight 0.21
  roadBorderWidth [
    0.14
  ]
  wayPoints [
    -105 562.2 0.048
    -105 486 0.048
  ]
  startLine [
    ""
    ""
    ""
    ""
    "{tex_stop}"
    "{tex_stop}"
    "{tex_stop}"
    "{tex_stop}"
  ]
  endLine [
    ""
    ""
    ""
    ""
    ""
    ""
    ""
    ""
  ]
  splineSubdivision -1
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}"""
        elif eid in ("ND_LOCAL_W", "ND_OUTER_SW", "ND_SHORE_W"):
            r = roads_n[eid]
            wps_str = "\n    ".join(f"{fmt(p[0])} {fmt(p[1])} {fmt(z)}" for p, z in zip(r["points"], r["elevations"]))
            n.set("wayPoints", f"[\n    {wps_str}\n  ]")
        elif name == "ND_Median-MAIN_S":
            n.text = f"""Solid {{
  translation 105 554.25 0.348
  children [
    Shape {{
      appearance {PAVEMENT}
      geometry Box {{
        size 0.6 148.5 0.6
      }}
    }}
  ]
  name "ND_Median-MAIN_S"
  boundingObject Box {{
    size 0.6 148.5 0.6
  }}
}}"""
        elif name == "ND_StopSign-WS_T":
            n.set("translation", "-123.6 567.9 0")

    extra_nd_wbt = []

    # 4 Split Junctions in North District
    extra_nd_wbt.append(nd_crossroad_wbt("ND_SPLIT_RIVER_E", "ND_Junction-SPLIT_RIVER_E", [], ["ND_MAIN_S", "ND_RIVER_BRIDGE_E", "ND_RIVER_E_GROUND_W", "ND_RIVER_E_GROUND_E"]))
    extra_nd_wbt.append(nd_crossroad_wbt("ND_SPLIT_RIVER_ES", "ND_Junction-SPLIT_RIVER_ES", [], ["ND_RIVER_BRIDGE_E"]))
    extra_nd_wbt.append(nd_crossroad_wbt("ND_SPLIT_RIVER_W", "ND_Junction-SPLIT_RIVER_W", [], ["ND_ROUND_SOUTH_LINK", "ND_RIVER_BRIDGE_W", "ND_RIVER_W_GROUND_W", "ND_RIVER_W_GROUND_E"]))
    extra_nd_wbt.append(nd_crossroad_wbt("ND_SPLIT_RIVER_WS", "ND_Junction-SPLIT_RIVER_WS", [], ["ND_RIVER_BRIDGE_W"]))

    # 2 River Bridges (Road nodes with roadBorderHeight 1.5, roadBorderWidth [0.6], 4 lanes)
    for rid, fj, tj, pts3 in [
        ("ND_RIVER_BRIDGE_E", "ND_SPLIT_RIVER_ES", "ND_SPLIT_RIVER_E", pts_bridge_e),
        ("ND_RIVER_BRIDGE_W", "ND_SPLIT_RIVER_WS", "ND_SPLIT_RIVER_W", pts_bridge_w),
    ]:
        wps_str = ",\n    ".join(f"{fmt(p[0])} {fmt(p[1])} {fmt(p[2])}" for p in pts3)
        extra_nd_wbt.append(f"""Road {{
  name "Road-{rid}"
  id "{rid}"
  startJunction "{fj}"
  endJunction "{tj}"
  width 14.4
  numberOfLanes 4
  numberOfForwardLanes 2
  lines [
    RoadLine {{
    }}
    RoadLine {{
      color 0.85 0.75 0.3
      type "none"
    }}
    RoadLine {{
    }}
  ]
  roadBorderHeight 1.5
  roadBorderWidth [
    0.6
  ]
  wayPoints [
    {wps_str}
  ]
  startLine [
    ""
    ""
    ""
    ""
  ]
  endLine [
    ""
    ""
    ""
    ""
  ]
  splineSubdivision -1
  appearance {ASPHALT}
  pavementAppearance {PARAPET_APP}
  roadBoundingObject TRUE
  rightBorderBoundingObject TRUE
  leftBorderBoundingObject TRUE
}}""")

    # 4 Outer 2-Lane Ground Roads on North Bank
    for rid, fj, tj, pts2, fwd, is_sc in [
        ("ND_RIVER_E_GROUND_W", "ND_J_SC_T", "ND_SPLIT_RIVER_E", [[94.2, 439.9062], [94.2, 480.0]], 0, True),
        ("ND_RIVER_E_GROUND_E", "ND_J_SC_T", "ND_SPLIT_RIVER_E", [[115.8, 439.9062], [115.8, 480.0]], 2, True),
        ("ND_RIVER_W_GROUND_W", "ND_J_SW_T", "ND_SPLIT_RIVER_W", [[-115.8, 445.7412], [-115.8, 486.0]], 0, False),
        ("ND_RIVER_W_GROUND_E", "ND_J_SW_T", "ND_SPLIT_RIVER_W", [[-94.2, 445.7412], [-94.2, 486.0]], 2, False),
    ]:
        if fwd == 0:
            border_off = "rightBorder FALSE"
            bo_on = "leftBorderBoundingObject TRUE"
            st_paint = tex_stop_cross if is_sc else tex_stop
            sl = f'[\n    "{st_paint}"\n    "{st_paint}"\n  ]'
        else:
            border_off = "leftBorder FALSE"
            bo_on = "rightBorderBoundingObject TRUE"
            sl = f'[\n    "{tex_cross}"\n    "{tex_cross}"\n  ]' if is_sc else '[\n    ""\n    ""\n  ]'
        extra_nd_wbt.append(f"""Road {{
  name "Road-{rid}"
  id "{rid}"
  startJunction "{fj}"
  endJunction "{tj}"
  width 7.2
  numberOfForwardLanes {fwd}
  lines [
    RoadLine {{
      width 0.15
    }}
  ]
  roadBorderHeight 0.21
  roadBorderWidth [
    0.14
  ]
  {border_off}
  wayPoints [
    {fmt(pts2[0][0])} {fmt(pts2[0][1])} 0.048
    {fmt(pts2[1][0])} {fmt(pts2[1][1])} 0.048
  ]
  startLine {sl}
  endLine [
    ""
    ""
  ]
  splineSubdivision -1
  appearance {ASPHALT}
  pavementAppearance {PAVEMENT}
  {bo_on}
}}""")

    # Median for widened 8-lane ND_ROUND_SOUTH_LINK
    extra_nd_wbt.append(f"""Solid {{
  translation -105 524.1 0.348
  children [
    Shape {{
      appearance {PAVEMENT}
      geometry Box {{
        size 0.6 76.2 0.6
      }}
    }}
  ]
  name "ND_Median-ROUND_SOUTH_LINK"
  boundingObject Box {{
    size 0.6 76.2 0.6
  }}
}}""")

    # 2-Manifold Decks, Ramp Supports, High Side Parapets, Median Plinths, Median Columns & River Piers for both bridges
    for tag, rid, pts3, pier_spec in [
        ("E", "RIVER_BRIDGE_E", pts_bridge_e, [(16, -0.10), (25, -1.50), (33, -1.50), (41, -1.50), (49, -0.10)]),
        ("W", "RIVER_BRIDGE_W", pts_bridge_w, [(16, -0.10), (24, -0.10), (33, -1.50), (41, -1.50), (49, -0.10)]),
    ]:
        v_dk, f_dk = build_curved_ribbon_mesh(pts3[6:-6], -7.2, 7.2, lambda j, z: z - 0.42, lambda j, z: z - 0.036)
        extra_nd_wbt.append(mesh_to_vrml(f"ND_RiverBridge{tag}-Deck", f"ND_MESH_RIVER_BRIDGE_{tag}_DECK", PAVEMENT, v_dk, f_dk))

        v_rs, f_rs = build_curved_ribbon_mesh(pts3[:7], -7.2, 7.2, lambda j, z: 0.0, lambda j, z: z - 0.036)
        extra_nd_wbt.append(mesh_to_vrml(f"ND_RiverBridge{tag}-RampSupport-South", f"ND_MESH_RIVER_BRIDGE_{tag}_RAMP_S", PAVEMENT, v_rs, f_rs))

        v_rn, f_rn = build_curved_ribbon_mesh(pts3[-7:], -7.2, 7.2, lambda j, z: 0.0, lambda j, z: z - 0.036)
        extra_nd_wbt.append(mesh_to_vrml(f"ND_RiverBridge{tag}-RampSupport-North", f"ND_MESH_RIVER_BRIDGE_{tag}_RAMP_N", PAVEMENT, v_rn, f_rn))

        v_pw, f_pw = build_curved_ribbon_mesh(pts3, -7.8, -7.2, lambda j, z: max(0.0, z - 0.42), lambda j, z: z + 1.50)
        extra_nd_wbt.append(mesh_to_vrml(f"ND_RiverBridge{tag}-Parapet-West", f"ND_MESH_RIVER_BRIDGE_{tag}_PARAPET_W", PARAPET_APP, v_pw, f_pw))

        v_pe, f_pe = build_curved_ribbon_mesh(pts3, 7.2, 7.8, lambda j, z: max(0.0, z - 0.42), lambda j, z: z + 1.50)
        extra_nd_wbt.append(mesh_to_vrml(f"ND_RiverBridge{tag}-Parapet-East", f"ND_MESH_RIVER_BRIDGE_{tag}_PARAPET_E", PARAPET_APP, v_pe, f_pe))

        v_mb, f_mb = build_curved_ribbon_mesh(pts3, -0.20, 0.20, lambda j, z: z, lambda j, z: z + 0.14)
        extra_nd_wbt.append(mesh_to_vrml(f"ND_Median-{rid}", f"ND_MESH_MEDIAN_{rid}", PAVEMENT, v_mb, f_mb))

        extra_nd_wbt.append(build_column_median_solid(f"ND_MedianColumns-{rid}", pts3))
        extra_nd_wbt.append(build_bridge_piers_solid(f"ND_RiverBridge{tag}-Piers", pts3, pier_spec))

    new_nd_str = nd_str
    for n in reversed(nodes_n):
        new_nd_str = new_nd_str[:n.start] + n.text + new_nd_str[n.end:]
    new_nd_str = re.sub(r"\n{3,}", "\n\n", new_nd_str.strip()) + "\n\n" + "\n\n".join(extra_nd_wbt) + "\n\n"

    final_wbt = new_prefix_str + "# BEGIN NORTH_DISTRICT\n\n" + new_nd_str + "# END NORTH_DISTRICT" + suffix_str
    WBT_PATH.write_text(final_wbt, encoding="utf-8", newline="\n")
    MAP_S_PATH.write_text(json.dumps(map_s, indent=2) + "\n", encoding="utf-8", newline="\n")
    MAP_N_PATH.write_text(json.dumps(map_n, indent=2) + "\n", encoding="utf-8", newline="\n")

    print("Successfully updated my_world.wbt, map.json, and map-north.json!")

    # Compute new prefix SHA256 for check_north_district.py
    actual_prefix_bytes = final_wbt.split("# BEGIN NORTH_DISTRICT")[0].encode("utf-8")
    new_hash = hashlib.sha256(actual_prefix_bytes).hexdigest()
    print(f"New South District prefix SHA256: {new_hash}")


if __name__ == "__main__":
    main()
