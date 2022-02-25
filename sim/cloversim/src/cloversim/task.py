import os
import random
import numpy as np

TASK_PATH = "/tmp/task_generated"
WORLD_PATH = TASK_PATH + "/task.world"

if not os.path.exists(TASK_PATH):
  os.mkdir(TASK_PATH)
  os.mkdir(TASK_PATH + "/models")

SEED = 0
def load_seed():
  seed_str = ""
  with open("/home/clover/task_seed", "r") as f:
    seed_str = f.read()

  # Minecraft seeding algorithm
  # More: https://docs.oracle.com/javase/8/docs/api/java/lang/String.html#hashCode--
  try:
    SEED = int(seed_str)
  except:
    SEED = 1
    l = len(seed_str) - 1
    for i, s in enumerate(seed_str):
      SEED += ord(s) * (31**(l - i))

  SEED &= 0xFFFFFFFF  # Limit to 2**32 - 1
  random.seed(SEED)
  np.random.seed(SEED)

if os.exists("/home/clover/task_seed"):
  load_seed()