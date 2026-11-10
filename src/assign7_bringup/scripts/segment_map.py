#!/usr/bin/env python3
"""Create reproducible semantic map assets from the saved original PGM/YAML."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

from semantic_map import OccupancyMap, build_regions

COLORS = [(65, 145, 225), (245, 170, 65), (105, 190, 125)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--seeds', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    occupancy = OccupancyMap(args.map)
    config = yaml.safe_load(Path(args.seeds).read_text())
    data, labels = build_regions(occupancy, config['regions'], config['minimum_clearance_m'])
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output/'regions.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    cv2.imwrite(str(output/'region_labels.png'), labels)
    rgb = np.repeat(occupancy.image[:, :, None], 3, axis=2)
    for i, color in enumerate(COLORS, 1):
        rgb[labels == i] = color
    scale = 5
    map_image = Image.fromarray(rgb).resize((occupancy.width*scale, occupancy.height*scale), Image.Resampling.NEAREST)
    canvas = Image.new('RGB', (map_image.width, map_image.height+135), 'white')
    canvas.paste(map_image, (0,0))
    draw = ImageDraw.Draw(canvas)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 19) if Path(font_path).exists() else ImageFont.load_default()
    for i, region in enumerate(data['regions']):
        u, v = region['goal_pixel']
        x, y = int((u+.5)*scale), int((v+.5)*scale)
        draw.ellipse((x-10,y-10,x+10,y+10), fill='white', outline='black', width=2)
        draw.text((x+12, y-12), region['id'], fill='black', font=font, stroke_width=1)
        yy = map_image.height+10+i*36
        draw.rectangle((12,yy,34,yy+22), fill=COLORS[i])
        draw.text((46, yy-2), f"{region['id']} — {region['name']} | goal {region['goal'][:2]}", fill='black', font=font)
    canvas.save(output/'segmented_map.png')
    for region in data['regions']:
        print(f"{region['id']}: {region['name']}; goal={region['goal']}; clearance={region['clearance_m']} m")
    print(f'Đã lưu vùng và ảnh phân vùng: {output}')


if __name__ == '__main__':
    main()
