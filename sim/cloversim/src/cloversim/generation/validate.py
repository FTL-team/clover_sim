def validate_pose(pose):
  if type(pose) is tuple and len(pose) != 6 and len(pose) != 3:
    raise ValueError(
        "Pose must be tuple with length of 6 (x, y, z, roll, pitch, yaw) or length of 3 (x, y, z)"
    )


def validate_size(size):
  if type(size) is tuple and len(size) != 3:
    raise ValueError(
        "Size must be tuple with length of 3 (width, height, depth)")
