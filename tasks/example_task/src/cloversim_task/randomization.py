#!/usr/bin/env python3

import random
import string
from .colors import colors

color_markers = list(colors.keys())
random.shuffle(color_markers)

status_markers = [random.random() < 0.5 for _ in range(5)]

qrcode_contents = random.choices(string.ascii_letters + string.digits, k=32)
qrcode_contents = ''.join(qrcode_contents)