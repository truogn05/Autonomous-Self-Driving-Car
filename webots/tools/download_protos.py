"""Download the requested Webots R2025a PROTOs, keeping upstream folders.

Run from the repository root: python webots/tools/download_protos.py
Only the listed PROTOs are downloaded; textures, meshes and dependencies are
not included. Existing files are preserved so local edits are not overwritten.
"""

from pathlib import Path
import sys
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1] / "protos" / "vendor"
BASE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/"
PROTO_GROUPS = {
    "projects/objects/road/protos": (
        "AddLaneRoadSegment", "AddLanesRoadSegment", "CrashBarrier",
        "Crossroad", "CurvedRoadSegment", "HelicoidalRoadSegment",
        "LaneSeparation", "Road", "RoadIntersection", "RoadLine",
        "RoadPillars", "Roundabout", "StraightRoadSegment",
    ),
    "projects/objects/traffic/protos": (
        "CautionPanel", "CautionSign", "ControlledStreetLight",
        "CrossRoadsTrafficLight", "DirectionPanel", "DivergentIndicator",
        "ExitPanel", "ExitSign", "GenericTrafficLight", "HighwayPole",
        "HighwaySign", "OrderPanel", "OrderSign", "ParkingLines",
        "ParkingMeter", "PedestrianCrossing", "Pole", "RectangularPanel",
        "SignPole", "SpeedLimitPanel", "SpeedLimitSign", "StopPanel",
        "StopSign", "StreetLight", "TrafficCone", "TrafficLightBigPole",
        "WorkBarrier", "YieldPanel", "YieldSign","TrafficLight","TrafficLightStandardLampGeometry",
        "TrafficLightHorizontal", "TrafficLightArrowLampGeometry", 
    ),
    "projects/objects/trees/protos": (
        "BigSassafras", "Cypress", "Forest", "Oak", "PalmTree", "Pine",
        "Sassafras", "SimpleTree", "Tree",
    ),
    "projects/vehicles/protos/abstract": ("Car",),
    "projects/vehicles/protos/bmw": ("BmwX5", "BmwX5Simple"),
    "projects/vehicles/protos/citroen": ("CitroenCZero", "CitroenCZeroSimple"),
    "projects/vehicles/protos/generic": (
        "Bus", "BusSimple", "MotorbikeSimple", "ScooterSimple", "Tractor",
        "Truck", "TruckSimple",
    ),
    "projects/vehicles/protos/lincoln": ("LincolnMKZ", "LincolnMKZSimple"),
    "projects/vehicles/protos/mercedes_benz": (
        "MercedesBenzSprinter", "MercedesBenzSprinterSimple",
    ),
    "projects/vehicles/protos/range_rover": (
        "RangeRoverSportSVR", "RangeRoverSportSVRSimple",
    ),
    "projects/vehicles/protos/tesla": ("TeslaModel3", "TeslaModel3Simple"),
    "projects/vehicles/protos/toyota": ("ToyotaPrius", "ToyotaPriusSimple"),
    "projects/appearances/protos": ("RoughConcrete",),
    "projects/objects/geometries/protos": ("TexturedBox",),
}


def main():
    downloaded = skipped = 0
    for folder, names in PROTO_GROUPS.items():
        for name in names:
            path = f"{folder}/{name}.proto"
            target = ROOT / path
            if target.is_file():
                skipped += 1
                print(f"SKIP {path}", flush=True)
                continue
            print(f"GET  {path}", flush=True)
            with urlopen(BASE + path, timeout=30) as response:
                data = response.read()
            if not data.startswith(b"#VRML_SIM R2025a utf8"):
                raise ValueError(f"Not a Webots R2025a PROTO: {path}")
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_suffix(".proto.part")
            temporary.write_bytes(data)
            temporary.replace(target)
            downloaded += 1
    print(f"Ready: {downloaded} downloaded, {skipped} existing, "
          f"{downloaded + skipped} total.\nFolder: {ROOT}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        print(f"Download failed: {error}\nRerun to resume; existing files are kept.",
              file=sys.stderr)
        sys.exit(1)
