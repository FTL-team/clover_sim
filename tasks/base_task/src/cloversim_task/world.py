#!/usr/bin/env python3

from cloversim.generation import World, Include
from cloversim.generation import ArucoMap, generate_aruco_map

WORLD = World()
WORLD.add(Include("model://sun"))
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

WORLD.add(ArucoMap("aruco_map", generate_aruco_map()).generate())

# Write your world generation code here