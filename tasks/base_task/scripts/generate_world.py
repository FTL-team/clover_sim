#!/usr/bin/env python3

from cloversim.generation import World, Include, Box, ColorMaterial, ImageTextures
import cv2
import numpy as np
import qrcode

world = World()
world.add(Include("model://sun"))
world.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))
world.add(Include("model://aruco_cmit_txt"))

colors = [(0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), (1, 0, 0), (1, 0, 1),
          (1, 1, 0), (1, 1, 1), (0.5, 0.5, 0.5), (1, 0.5, 0)]

for i, color in zip(range(10), colors):
  world.add(
      Box("color_box_" + str(i),
          size=(0.4, 0.4, 0.1),
          mass=1,
          pose=(i, -1, 0.05),
          material=ColorMaterial(color)))

world.add(
    Box(
        "multi_color_box",
        size=(2, 1, 0.5),
        pose=(-2.5, -2.5, 0.25),
        material=(tuple([ColorMaterial(color) for color in colors[1:7]])),
    ))

world.add(Include("model://example_model", pose=(-2.5, -2.5, 3)))


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
  world.add(
      Box(
          "status_marker_" + str(i),
          size=(0.8, 0.8, 0.001),
          pose=(-1, i * 2 + 1, 0.001),
          material=image_textures.materials[
              "marker_" + ("ok" if marker_status else "error")],
      ))

world.add(
    Box("qrcode",
        size=(1, 1, 0.001),
        pose=(-1, -1, 0.001),
        mass=0.1,
        material=image_textures["qrcode"]))

world.save()
