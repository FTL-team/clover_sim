import os
import random
import numpy as np

TASK_PATH = "/tmp/task_generated"
WORLD_PATH = TASK_PATH + "/task.world"

if not os.path.exists(TASK_PATH):
  os.mkdir(TASK_PATH)
  os.mkdir(TASK_PATH + "/models")

RANDOMIZATION = 0
def load_rand():
  global RANDOMIZATION

  rand_str = ""
  with open("/home/clover/task_randomization", "r") as f:
    rand_str = f.read()

  # Minecraft seeding algorithm
  # More: https://docs.oracle.com/javase/8/docs/api/java/lang/String.html#hashCode--
  try:
    RANDOMIZATION = int(rand_str)
  except:
    RANDOMIZATION = 1
    l = len(rand_str) - 1
    for i, s in enumerate(rand_str):
      RANDOMIZATION += ord(s) * (31**(l - i))

  RANDOMIZATION &= 0xFFFFFFFF  # Limit to 2**32 - 1
  random.seed(RANDOMIZATION)
  np.random.seed(RANDOMIZATION)

if os.path.exists("/home/clover/task_randomization"):
  load_rand()