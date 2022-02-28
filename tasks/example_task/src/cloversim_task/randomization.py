#!/usr/bin/env python3

import random
import string
from .colors import colors
from cloversim.randomization import create_random_string, randfloat, randbool, create2d_postions

color_markers = [
    'blue', 'green', 'cyan', 'red', 'magenta', 'yellow', 'gray', 'orange'
]
random.shuffle(color_markers)

status_markers = [randbool() for _ in range(5)]

qrcode_contents = create_random_string(32)

random_object_posA = randfloat(12, 18)
random_object_posB = randfloat(12, 18)

marker_positions = create2d_postions(
    [
        1,  # Landing pad
        .5,
        .5,
        .5,
        .5,
        .5  # 5 Status markers
    ],
    (8, 8),
    (0.5, 0.5))
