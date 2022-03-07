import random
import string

def create_random_string(length):
  """
  Creates random string that contains ascii letters and digits.
  """
  return ''.join(random.choices(string.ascii_letters + string.digits, k=length))

def randfloat(lower_bound, upper_bound):
  """
  Generates random float in range [lower_bound, upper_bound)
  """
  return random.random() * (upper_bound - lower_bound) + lower_bound


def randbool(p=0.5):
  """
  Generates random boolean, bool will be True with probability p
  """
  return random.random() >= p

def create2d_postions(objs_radius, field_size, field_start=(0,0)):
  def dist(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

  for try_i in range(2**16):
    positions = [(randfloat(0, field_size[0]) + field_start[0],
                  randfloat(0, field_size[1]) + field_start[1])
                 for obj in objs_radius]
    ok = True
    for i, rad_a in enumerate(objs_radius):
      for j, rad_b in enumerate(objs_radius):
        if i != j:
          if dist(positions[i], positions[j]) < (rad_a + rad_b) ** 2:
            ok = False
            break
    if ok:
      break
  else:
    raise TimeoutError("Could not generate positions")
  return positions
