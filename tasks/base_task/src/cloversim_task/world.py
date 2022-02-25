#!/usr/bin/env python3

from cloversim.generation import World, Include, Box, Cylinder
from cloversim.generation import ColorMaterial, ImageTextures, ArucoMap, generate_aruco_map
import cv2
import numpy as np
import qrcode

WORLD = World()
WORLD.add(Include("model://sun"))
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

WORLD.add(ArucoMap("aruco_map", generate_aruco_map()).generate())

colors = [(0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), (1, 0, 0), (1, 0, 1),
          (1, 1, 0), (1, 1, 1), (0.5, 0.5, 0.5), (1, 0.5, 0)]

for i, color in zip(range(10), colors):
  WORLD.add(
      Box("color_box_" + str(i),
          size=(0.4, 0.4, 0.1),
          mass=1,
          pose=(i, -1, 0.05),
          material=ColorMaterial(color),
          static=True))

WORLD.add(
    Box(
        "multi_color_box",
        size=(2, 1, 0.5),
        pose=(-2.5, -2.5, 0.25),
        material=(tuple([ColorMaterial(color) for color in colors[1:7]])),
    ))

WORLD.add(Include("model://example_model", pose=(-2.5, -2.5, 3)))


def generate_marker(marker_color):
  img = np.zeros((256, 256, 3), np.uint8)
  img[:] = (255, 0, 0)  # Fill with blue color
  cv2.circle(img, (128, 128), 96, marker_color, -1)
  return img


image_textures = ImageTextures({
    "qrcode": qrcode.make("orange").get_image(),
    "marker_ok": generate_marker((0, 255, 0)),
    "marker_error": generate_marker((0, 0, 255))
})
image_textures.generate_materials()

markers_status = [True, True, False, True, False]
for i, marker_status in enumerate(markers_status):
  WORLD.add(
      Box(
          "status_marker_" + str(i),
          size=(0.8, 0.8, 0.001),
          pose=(-1, i * 2 + 1, 0.001),
          material=image_textures.materials[
              "marker_" + ("ok" if marker_status else "error")],
          static=True
      ))

WORLD.add(
    Box("qrcode",
        size=(1, 1, 0.001),
        pose=(-1, -1, 0.001),
        mass=0.1,
        material=image_textures["qrcode"],
        static=True))

WORLD.add(
    Cylinder("landing_pad",
             radius=0.5,
             length=0.3,
             pose=(10, 10, 0.15),
             material=(
                 ColorMaterial((0.5, 0.5, 0.5)),
                 ColorMaterial((1, 0.5, 0)),
                 ColorMaterial((1, 1, 0)),
)))
