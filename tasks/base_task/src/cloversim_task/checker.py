#!/usr/bin/env python3

# Write your checker there

from cloversim.utils import distance_between_points
from .randomization import *
from cloversim.score import ScoreTask, Scoring
from cloversim.checker import get_clover_position, is_clover_armed, is_clover_still
import rospy

landing = ScoreTask('Land', 10)

target_points = [
    (0, 0, 1),
    (1, 0, 1),
    (1, 1, 1),
    (0, 1, 1),
]

point_tasks = [
    ScoreTask('Point A', 10),
    ScoreTask('Point B', 10),
    ScoreTask('Point C', 10),
    ScoreTask('Point D', 10),
]

scoring = Scoring('Fly rectangle task', [*point_tasks, landing])


def mark_point(point, completed):
  if point_tasks[point].failed:
    return
  if not completed:
    point_tasks[point].mark_failed()
    point_tasks[point].set_score(0)
  else:
    point_tasks[point].set_score(10)


current_point = 0
last_land_position = (0, 0, 0)

def process_position():
  global current_point
  global last_land_position

  pos = get_clover_position()
  is_still = is_clover_still(pos)
  armed = is_clover_armed()

  position = pos.pose.position
  x, y, z = position.x, position.y, position.z

  if is_still and not armed:
    last_land_position = (x, y, z)
    if distance_between_points((x, y), (0, 0)) < 0.2:
      if not landing.failed:
        landing.set_score(10)
    else:
      landing.set_score(0)
      landing.mark_failed()
  else:
    landing.set_score(0)

  nearest_point = -1
  cur_dist = 0
  
  # Calculate current position relative to takeoff position
  x -= last_land_position[0]
  y -= last_land_position[1]
  z -= last_land_position[2]

  # If we are close enough on height then we can ignore it
  if abs(x - 1) < 0.2:
    z = 1

  
  for i, p in enumerate(target_points):
    d = distance_between_points(p, (x, y, z))
    if d < cur_dist or cur_dist == 0:
      nearest_point = i
      cur_dist = d
  
  if nearest_point == current_point or nearest_point == (current_point - 1) % 4:
    # Too far from point
    if cur_dist > 1.5:
      mark_point(nearest_point, False)
    elif cur_dist < 0.2:
      current_point = (nearest_point + 1) % 4
      mark_point(nearest_point, True)
  else:
    # Wrong order of points
    for i in range(4):
      mark_point(i, False)


while True:
  try:
    process_position()
  except Exception as E:
    print(E)
    pass
  rospy.sleep(0.2)