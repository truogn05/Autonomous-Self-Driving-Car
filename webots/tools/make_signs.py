"""Small, legible local textures for traffic signs, backing plates, and road markings."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math

out = Path(__file__).resolve().parents[1] / 'worlds' / 'textures'
out.mkdir(exist_ok=True)
font = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 96)
font_lg = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 150)
small = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', 48)

# 1. Core active world signs (stop, yield, roundabout, speed)
for kind in ['stop', 'yield', 'roundabout', 'speed']:
    im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    if kind == 'stop':
        pts = [(256 + 240 * math.cos(math.pi / 8 + i * math.pi / 4), 256 + 240 * math.sin(math.pi / 8 + i * math.pi / 4)) for i in range(8)]
        d.polygon(pts, fill='#faf8f0')
        pts = [(256 + 221 * math.cos(math.pi / 8 + i * math.pi / 4), 256 + 221 * math.sin(math.pi / 8 + i * math.pi / 4)) for i in range(8)]
        d.polygon(pts, fill='#ce272f')
        d.text((256, 256), 'STOP', font=font, anchor='mm', fill='white')
    elif kind == 'yield':
        d.polygon([(20, 46), (492, 46), (256, 484)], fill='#d72535')
        d.polygon([(85, 88), (427, 88), (256, 405)], fill='#faf8ef')
    elif kind == 'roundabout':
        d.ellipse((14, 14, 498, 498), fill='white')
        d.ellipse((30, 30, 482, 482), fill='#1461c0')
        for i in range(3):
            a = i * 120
            d.arc((125, 125, 387, 387), a + 15, a + 100, fill='white', width=25)
            t = math.radians(a + 15)
            x = 256 + 131 * math.cos(t)
            y = 256 + 131 * math.sin(t)
            u = (math.sin(t), -math.cos(t))
            n = (-u[1], u[0])
            d.polygon([(x + 35 * u[0], y + 35 * u[1]), (x - 25 * u[0] + 32 * n[0], y - 25 * u[1] + 32 * n[1]), (x - 25 * u[0] - 32 * n[0], y - 25 * u[1] - 32 * n[1])], fill='white')
    else:
        d.ellipse((15, 15, 497, 497), fill='#cf2535')
        d.ellipse((53, 53, 459, 459), fill='white')
        d.text((256, 225), '0.5', font=font, anchor='mm', fill='#20252a')
        d.text((256, 330), 'm/s', font=small, anchor='mm', fill='#20252a')
    im.save(out / (kind + '.png'))

# 2. Exact-silhouette metallic backing textures (prevents square Box artifacts & Z-fighting)
for shape in ['octagon', 'triangle', 'circle']:
    im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    if shape == 'octagon':
        pts_out = [(256 + 240 * math.cos(math.pi / 8 + i * math.pi / 4), 256 + 240 * math.sin(math.pi / 8 + i * math.pi / 4)) for i in range(8)]
        pts_in = [(256 + 226 * math.cos(math.pi / 8 + i * math.pi / 4), 256 + 226 * math.sin(math.pi / 8 + i * math.pi / 4)) for i in range(8)]
        d.polygon(pts_out, fill='#5c6168')
        d.polygon(pts_in, fill='#7c828a')
    elif shape == 'triangle':
        # Note: Back face has U flipped horizontally relative to front, symmetric along X=256
        d.polygon([(20, 46), (492, 46), (256, 484)], fill='#5c6168')
        d.polygon([(38, 58), (474, 58), (256, 462)], fill='#7c828a')
    elif shape == 'circle':
        d.ellipse((14, 14, 498, 498), fill='#5c6168')
        d.ellipse((28, 28, 484, 484), fill='#7c828a')
    im.save(out / f'sign_back_{shape}.png')

print('Core sign textures and exact-silhouette backing textures written.')

# 3. Additional prepared commonly-used traffic signs (saved in textures/ for future use)
def make_prohibitory_base():
    im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.ellipse((14, 14, 498, 498), fill='white')
    d.ellipse((22, 22, 490, 490), fill='#d72535')
    d.ellipse((70, 70, 442, 442), fill='#faf8f0')
    return im, d

def draw_red_slash(d):
    # Diagonal slash from top-left to bottom-right
    angle = math.radians(45)
    dx = 22 * math.cos(angle)
    dy = 22 * math.sin(angle)
    p1, p2 = (115, 115), (397, 397)
    d.polygon([
        (p1[0] - dx, p1[1] + dy),
        (p1[0] + dx, p1[1] - dy),
        (p2[0] + dx, p2[1] - dy),
        (p2[0] - dx, p2[1] + dy),
    ], fill='#d72535')

def make_mandatory_base():
    im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.ellipse((14, 14, 498, 498), fill='white')
    d.ellipse((28, 28, 484, 484), fill='#1461c0')
    return im, d

# 3.1. no_entry.png (Cấm đi ngược chiều)
im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
d = ImageDraw.Draw(im)
d.ellipse((14, 14, 498, 498), fill='white')
d.ellipse((26, 26, 486, 486), fill='#d72535')
d.rounded_rectangle((76, 212, 436, 300), radius=18, fill='white')
im.save(out / 'no_entry.png')

# 3.2. no_left_turn.png (Cấm rẽ trái)
im, d = make_prohibitory_base()
# Stem going up, bending left
d.rectangle((268, 210, 316, 385), fill='#1e2226')
d.rectangle((180, 186, 316, 234), fill='#1e2226')
d.polygon([(125, 210), (195, 150), (195, 270)], fill='#1e2226')
draw_red_slash(d)
im.save(out / 'no_left_turn.png')

# 3.3. no_right_turn.png (Cấm rẽ phải)
im, d = make_prohibitory_base()
d.rectangle((196, 210, 244, 385), fill='#1e2226')
d.rectangle((196, 186, 332, 234), fill='#1e2226')
d.polygon([(387, 210), (317, 150), (317, 270)], fill='#1e2226')
draw_red_slash(d)
im.save(out / 'no_right_turn.png')

# 3.4. no_u_turn.png (Cấm quay đầu xe)
im, d = make_prohibitory_base()
d.arc((156, 135, 356, 335), 180, 360, fill='#1e2226', width=44)
d.rectangle((312, 235, 356, 385), fill='#1e2226')
d.rectangle((156, 235, 200, 315), fill='#1e2226')
d.polygon([(178, 375), (126, 305), (230, 305)], fill='#1e2226')
draw_red_slash(d)
im.save(out / 'no_u_turn.png')

# 3.5. no_parking.png (Cấm đỗ xe)
im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
d = ImageDraw.Draw(im)
d.ellipse((14, 14, 498, 498), fill='white')
d.ellipse((22, 22, 490, 490), fill='#d72535')
d.ellipse((74, 74, 438, 438), fill='#1461c0')
draw_red_slash(d)
im.save(out / 'no_parking.png')

# 3.6. speed_30.png, speed_40.png, speed_50.png, speed_60.png
for spd in [30, 40, 50, 60]:
    im, d = make_prohibitory_base()
    d.text((256, 260), str(spd), font=font_lg, anchor='mm', fill='#1e2226')
    im.save(out / f'speed_{spd}.png')

# 3.7. straight_only.png (Hướng đi thẳng phải theo)
im, d = make_mandatory_base()
d.rectangle((228, 175, 284, 420), fill='white')
d.polygon([(256, 75), (168, 195), (344, 195)], fill='white')
im.save(out / 'straight_only.png')

# 3.8. turn_left_only.png (Hướng rẽ trái phải theo)
im, d = make_mandatory_base()
d.rectangle((264, 215, 318, 415), fill='white')
d.rectangle((175, 190, 318, 244), fill='white')
d.polygon([(95, 217), (190, 142), (190, 292)], fill='white')
im.save(out / 'turn_left_only.png')

# 3.9. turn_right_only.png (Hướng rẽ phải phải theo)
im, d = make_mandatory_base()
d.rectangle((194, 215, 248, 415), fill='white')
d.rectangle((194, 190, 337, 244), fill='white')
d.polygon([(417, 217), (322, 142), (322, 292)], fill='white')
im.save(out / 'turn_right_only.png')

# 3.10. keep_right.png (Hướng phải đi vòng sang phải)
im, d = make_mandatory_base()
# Diagonal arrow pointing bottom-right (45 deg)
d.line([(145, 145), (335, 335)], fill='white', width=54)
d.polygon([(385, 385), (375, 250), (250, 375)], fill='white')
im.save(out / 'keep_right.png')

# 3.11. pedestrian_crossing_sign.png (Biển đường người đi bộ sang ngang)
im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
d = ImageDraw.Draw(im)
d.rounded_rectangle((16, 16, 496, 496), radius=36, fill='white')
d.rounded_rectangle((30, 30, 482, 482), radius=28, fill='#1461c0')
d.polygon([(256, 68), (64, 430), (448, 430)], fill='white')
# Zebra stripes at bottom of triangle
for zx in range(118, 390, 46):
    d.polygon([(zx, 375), (zx + 24, 375), (zx + 30, 418), (zx - 6, 418)], fill='#1e2226')
# Simple stylized pedestrian figure
d.ellipse((236, 152, 276, 192), fill='#1e2226')  # head
d.line([(256, 195), (246, 280)], fill='#1e2226', width=22)  # torso
d.line([(246, 275), (210, 365)], fill='#1e2226', width=18)  # left leg
d.line([(246, 275), (295, 365)], fill='#1e2226', width=18)  # right leg
d.line([(252, 215), (210, 265)], fill='#1e2226', width=16)  # left arm
d.line([(252, 215), (296, 255)], fill='#1e2226', width=16)  # right arm
im.save(out / 'pedestrian_crossing_sign.png')

print('13 additional commonly-used traffic sign textures generated.')

# 4. Native Road.startLine/endLine textures (512x512 high resolution)
for kind in ['transparent', 'stop_line', 'crossing', 'stop_crossing']:
    im = Image.new('RGBA', (512, 512), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    if kind == 'stop_line':
        d.rectangle((0, 430, 511, 480), fill=(250, 250, 240, 255))
    elif kind == 'crossing':
        for x in range(0, 512, 128):
            d.rectangle((x, 140, x + 64, 490), fill=(250, 250, 240, 255))
    elif kind == 'stop_crossing':
        d.rectangle((0, 32, 511, 80), fill=(250, 250, 240, 255))
        for x in range(0, 512, 128):
            d.rectangle((x, 140, x + 64, 490), fill=(250, 250, 240, 255))
    im.save(out / (kind + '.png'))

print('Road marking textures written (stop_line, crossing, stop_crossing, transparent).')
