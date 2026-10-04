"""Small, legible local textures for the miniature traffic signs."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math

out=Path(__file__).resolve().parents[1]/'worlds'/'textures'
out.mkdir(exist_ok=True)
font=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',96)
small=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',48)
for kind in ['stop','yield','roundabout','speed']:
    im=Image.new('RGBA',(512,512),(0,0,0,0)); d=ImageDraw.Draw(im)
    if kind=='stop':
        pts=[(256+240*math.cos(math.pi/8+i*math.pi/4),256+240*math.sin(math.pi/8+i*math.pi/4)) for i in range(8)]
        d.polygon(pts,fill='#faf8f0')
        pts=[(256+221*math.cos(math.pi/8+i*math.pi/4),256+221*math.sin(math.pi/8+i*math.pi/4)) for i in range(8)]
        d.polygon(pts,fill='#ce272f'); d.text((256,256),'STOP',font=font,anchor='mm',fill='white')
    elif kind=='yield':
        d.polygon([(20,46),(492,46),(256,484)],fill='#d72535')
        d.polygon([(85,88),(427,88),(256,405)],fill='#faf8ef')
    elif kind=='roundabout':
        d.ellipse((14,14,498,498),fill='white');d.ellipse((30,30,482,482),fill='#1461c0')
        for i in range(3):
            a=i*120
            d.arc((125,125,387,387),a+15,a+100,fill='white',width=25)
            t=math.radians(a+15); x=256+131*math.cos(t); y=256+131*math.sin(t)
            u=(math.sin(t),-math.cos(t)); n=(-u[1],u[0])
            d.polygon([(x+35*u[0],y+35*u[1]),(x-25*u[0]+32*n[0],y-25*u[1]+32*n[1]),(x-25*u[0]-32*n[0],y-25*u[1]-32*n[1])],fill='white')
    else:
        d.ellipse((15,15,497,497),fill='#cf2535');d.ellipse((53,53,459,459),fill='white')
        d.text((256,225),'0.5',font=font,anchor='mm',fill='#20252a')
        d.text((256,330),'m/s',font=small,anchor='mm',fill='#20252a')
    im.save(out/(kind+'.png'))
print('Four miniature sign textures written.')

# Native Road.startLine/endLine textures; no extra world geometry.
for kind in ['transparent','stop_line','crossing','stop_crossing']:
    im=Image.new('RGBA',(256,256),(0,0,0,0)); d=ImageDraw.Draw(im)
    if kind in ('stop_line','stop_crossing'):
        d.rectangle((0,208,255,239),fill=(250,250,240,255))
    if kind in ('crossing','stop_crossing'):
        for x in range(0,256,128):
            d.rectangle((x,16,x+53,143),fill=(250,250,240,255))
    im.save(out/(kind+'.png'))
