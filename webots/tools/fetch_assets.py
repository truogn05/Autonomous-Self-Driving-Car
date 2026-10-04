"""Vendor the R2025a PROTO dependency tree, preserving upstream licenses."""
from pathlib import Path
import posixpath
import re
import urllib.request

ROOT = Path(__file__).resolve().parents[1] / 'protos' / 'vendor'
BASE = 'https://raw.githubusercontent.com/cyberbotics/webots/R2025a/'
SEEDS = ['projects/objects/road/protos/' + name + '.proto' for name in
         ('StraightRoadSegment', 'Road', 'CurvedRoadSegment', 'Crossroad',
          'RoadIntersection', 'Roundabout', 'RoadLine')]
seen = set()


def fetch(path):
    path = posixpath.normpath(path)
    if path in seen:
        return
    seen.add(path)
    target = ROOT / path
    if target.exists():
        data = target.read_bytes()
    else:
        data = urllib.request.urlopen(BASE + path, timeout=60).read()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        print(path, len(data), flush=True)
    if not path.endswith('.proto'):
        return
    source = data.decode('utf-8')
    def replace(match):
        url = match.group(1)
        if '%<' in url or "'" in url or not re.search(r'\.(proto|png|jpg|jpeg|obj)$', url):
            return match.group(0)
        if url.startswith('webots://'):
            child = url[len('webots://'):]
        elif url.startswith(BASE):
            child = url[len(BASE):]
        elif '://' in url:
            return match.group(0)
        else:
            child = posixpath.normpath(posixpath.join(posixpath.dirname(path), url))
        fetch(child)
        return '"' + posixpath.relpath(child, posixpath.dirname(path)) + '"'
    source = re.sub(r'"([^"\n]+)"', replace, source)
    target.write_text(source, encoding='utf-8')


if __name__ == '__main__':
    for seed in SEEDS:
        fetch(seed)
    for channel in ('base_color', 'roughness', 'normal', 'occlusion'):
        fetch('projects/appearances/protos/textures/pavement/braun_square_stone_pavement_' + channel + '.jpg')
    print(f'{len(seen)} asset files ready in {ROOT}')
