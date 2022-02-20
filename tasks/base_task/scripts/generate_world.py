#!/usr/bin/env python3

from cloversim.generation import World, Include, Box, ColorMaterial

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
        pose=(-1.5, -1.5, 0.25),
        material=(tuple([ColorMaterial(color) for color in colors[1:7]])),
    ))

world.add(Include("model://example_model", pose=(-1.5, 0, 3)))

world.save()
