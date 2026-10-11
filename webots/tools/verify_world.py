"""Run the generated world in a separate minimized Webots process and retain its log."""
from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--webots', default='E:/ProgramE/Webots/msys64/mingw64/bin/webots.exe')
parser.add_argument('--port', default='1264')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
source = (root / 'worlds/my_world.wbt').read_text(encoding='utf-8')

world = root / 'worlds/my_world_check.wbt'
world_wbproj = root / 'worlds/.my_world_check.wbproj'
world.write_text(
    source.replace('controller "world_inspector"', 'controller "world_inspector" controllerArgs [ "--verify" ]'),
    encoding='utf-8',
)
(root / 'validation').mkdir(exist_ok=True)
try:
    with (root / 'validation/webots.log').open('w', encoding='utf-8') as log:
        proc = subprocess.run(
            [
                args.webots,
                '--batch',
                '--minimize',
                '--mode=fast',
                f'--port={args.port}',
                '--stdout',
                '--stderr',
                str(world),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            timeout=240,
        )
finally:
    if world.exists():
        try:
            world.unlink()
        except Exception:
            pass
    if world_wbproj.exists():
        try:
            world_wbproj.unlink()
        except Exception:
            pass

print((root / 'validation/webots.log').read_text(encoding='utf-8'))
raise SystemExit(proc.returncode)
