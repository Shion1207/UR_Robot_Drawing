"""ROS occupancy semantics, image/world geometry and seeded geodesic partition."""
from collections import deque
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class OccupancyMap:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).resolve()
        metadata = yaml.safe_load(self.yaml_path.read_text())
        self.image_path = (self.yaml_path.parent / metadata['image']).resolve()
        self.image = cv2.imread(str(self.image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError(f'Không đọc được ảnh map: {self.image_path}')
        self.height, self.width = self.image.shape
        self.resolution = float(metadata['resolution'])
        self.origin = tuple(float(v) for v in metadata['origin'])
        if self.resolution <= 0 or len(self.origin) != 3 or not all(math.isfinite(v) for v in (*self.origin, self.resolution)):
            raise ValueError('Resolution/origin không hợp lệ')
        probability = self.image.astype(np.float32)/255.0
        if not metadata.get('negate', 0):
            probability = 1.0-probability
        self.free = probability < float(metadata['free_thresh'])
        # Unknown occupancy is not traversable. Padding prevents boundary clearance artifacts.
        padded = np.pad(self.free.astype(np.uint8), 1)
        self.clearance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]*self.resolution
        self.fingerprint = hashlib.sha256(self.image_path.read_bytes()+self.yaml_path.read_bytes()).hexdigest()

    def pixel_to_world(self, u, v):
        x, y = (u+.5)*self.resolution, (self.height-v-.5)*self.resolution
        ox, oy, yaw = self.origin
        return ox+math.cos(yaw)*x-math.sin(yaw)*y, oy+math.sin(yaw)*x+math.cos(yaw)*y

    def world_to_pixel(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x-ox, y-oy
        lx, ly = math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy
        u, v = math.floor(lx/self.resolution), self.height-1-math.floor(ly/self.resolution)
        if not (0 <= u < self.width and 0 <= v < self.height):
            raise ValueError(f'Điểm ({x}, {y}) nằm ngoài bản đồ')
        return u, v


def build_regions(occupancy, seeds, clearance=0.4):
    if len(seeds) < 3 or len({r['id'] for r in seeds}) != len(seeds):
        raise ValueError('Cần ít nhất 3 vùng có ID khác nhau')
    safe = occupancy.free & (occupancy.clearance >= clearance)
    pixels = np.argwhere(safe)
    if not len(pixels):
        raise ValueError('Không có vùng trống đủ khoảng cách an toàn')
    labels = np.zeros(safe.shape, np.uint8)
    queue, regions = deque(), []
    for number, seed in enumerate(seeds, 1):
        u, v = occupancy.world_to_pixel(*seed['seed'])
        if not safe[v, u]:
            distances = (pixels[:, 0]-v)**2+(pixels[:, 1]-u)**2
            index = int(np.argmin(distances))
            if math.sqrt(float(distances[index]))*occupancy.resolution > 0.8:
                raise ValueError(f"Seed {seed['id']} quá xa vùng an toàn")
            v, u = map(int, pixels[index])
        if labels[v, u]:
            raise ValueError('Hai seed trùng ô đích')
        labels[v, u] = number
        queue.append((u, v, number))
        x, y = occupancy.pixel_to_world(u, v)
        regions.append(dict(id=seed['id'], name=seed['name'], aliases=seed.get('aliases', []),
                            goal=[round(x, 5), round(y, 5), float(seed.get('yaw', 0))],
                            goal_pixel=[u, v], clearance_m=round(float(occupancy.clearance[v, u]), 3)))
    while queue:
        u, v, number = queue.popleft()
        for nu, nv in ((u-1,v), (u+1,v), (u,v-1), (u,v+1)):
            if 0 <= nu < occupancy.width and 0 <= nv < occupancy.height and safe[nv, nu] and not labels[nv, nu]:
                labels[nv, nu] = number
                queue.append((nu, nv, number))
    for i, region in enumerate(regions, 1):
        region['area_m2'] = round(int((labels == i).sum())*occupancy.resolution**2, 3)
    return dict(schema_version=1, method='clearance-filtered multi-source geodesic Voronoi',
                source_sha256=occupancy.fingerprint, resolution=occupancy.resolution,
                origin=list(occupancy.origin), width=occupancy.width, height=occupancy.height,
                minimum_clearance_m=clearance, regions=regions), labels


def load_regions(path, map_path):
    data = json.loads(Path(path).read_text())
    occupancy = OccupancyMap(map_path)
    if data['source_sha256'] != occupancy.fingerprint:
        raise ValueError('Map đã thay đổi. Chạy scripts/segment_map.bash để tạo lại vùng.')
    for region in data['regions']:
        u, v = occupancy.world_to_pixel(*region['goal'][:2])
        if occupancy.clearance[v, u] < data['minimum_clearance_m'] or not occupancy.free[v, u]:
            raise ValueError(f"Điểm đích vùng {region['id']} không an toàn")
    return data, occupancy
