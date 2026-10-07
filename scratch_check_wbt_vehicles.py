import re

with open("webots/worlds/my_world.wbt", "r", encoding="utf-8") as f:
    text = f.read()

# Search for Robot, Car, vehicle in my_world.wbt
robots = re.findall(r'(?:DEF\s+\w+\s+)?(?:Robot|Car|BmwX5\w*)\s*\{[^{}]*name\s+"([^"]+)"', text)
print("Robots/Vehicles in my_world.wbt:", robots)

# Search for any car/vehicle references
lines = text.splitlines()
for idx, line in enumerate(lines):
    if any(k in line.lower() for k in ["bmw", "vehicle", "car", "controller"]):
        if not any(k in line for k in ["traffic_signals", "controller \"traffic_signals\""]):
            print(f"Line {idx+1}: {line}")
