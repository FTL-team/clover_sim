#!/usr/bin/env python3

from cloversim.utils import distance_between_points, get_servers_handler
from .randomization import *
from cloversim.score import ScoreTask, ScoreGroup, Scoring, create_update, FailMode
from cloversim.checker import get_clover_position, is_clover_armed, is_clover_still, match_cordinates
from xmlrpc.server import SimpleXMLRPCServer
import selectors

landing = ScoreGroup('Landing',
                     [ScoreTask('Landed', 3),
                      ScoreTask('On platform', 7)])

color_markers_task = ScoreTask('Color markers order', 10)

status_markers = []
for i in range(5):
  status_markers.append(
      ScoreGroup(f'Status marker {i+1}', [
          ScoreTask('Correct status', 0.6),
          ScoreTask('Correct position', 1.4),
      ]))  # Max 2 points per status marker, 10 in total


def status_marker_update(group, tasks):
  failed = sum(t.failed for t in tasks)
  if failed == 5:
    group.failed = True
    group.score = 0
  else:
    group.failed = False
    group.score = sum(t.score for t in tasks)


status_markers_group = ScoreGroup('Status markers',
                                  status_markers,
                                  update_function=create_update(FailMode.ALL))

qrcode_task = ScoreTask('QR code', 10)

scoring = Scoring(
    'Example tsak',
    [color_markers_task, status_markers_group, qrcode_task, landing])


class TaskServer(SimpleXMLRPCServer):
  qrcode_complete = False
  color_markers_complete = False
  status_markers_complete = False

  def __init__(self):
    SimpleXMLRPCServer.__init__(self, ('0.0.0.0', 8080))

    self.register_introspection_functions()

    self.register_function(self.report_qrcode)
    self.register_function(self.report_color_markers)
    self.register_function(self.report_status_markers)

  def report_qrcode(self, string):
    if self.qrcode_complete:
      return 0
    self.qrcode_complete = True

    if string == qrcode_contents:
      qrcode_task.set_score(10)
    else:
      qrcode_task.mark_failed()
    return 0

  def report_color_markers(self, order):
    if self.color_markers_complete:
      return 0
    self.color_markers_complete = True

    correct_order = 0
    for predicted, correct in zip(order, color_markers):
      if predicted == correct:
        correct_order += 1
    if correct_order == 0:
      color_markers_task.mark_failed()
    else:
      color_markers_task.set_score(10)

    return 0

  def report_status_markers(self, markers):
    if self.status_markers_complete:
      return 0
    self.status_markers_complete = True

    predicted_cords = [(marker[1], marker[2]) for marker in markers]
    idxs = match_cordinates(marker_positions[1:], predicted_cords)
    for correct_id, predicted_id in idxs:
      predicted_pos = predicted_cords[predicted_id]
      correct_pos = marker_positions[1 + correct_id]
      print("Matching", predicted_pos, "to", correct_pos, predicted_id,
            correct_id)

      dist = distance_between_points(predicted_pos, correct_pos)
      if dist > 0.5:
        status_markers[correct_id]['Correct status'].mark_failed()
        status_markers[correct_id]['Correct position'].mark_failed()
      else:
        if markers[predicted_id][2]:
          status_markers[correct_id]['Correct status'].set_score(0.6)
        else:
          status_markers[correct_id]['Correct status'].mark_failed()

        dist_score = (0.45 - max(dist - 0.05, 0)) / 0.45 * 1.4
        status_markers[correct_id]['Correct position'].set_score(dist_score)
    return 0


def check_landing():
  pos = get_clover_position()
  is_still = is_clover_still(pos)
  armed = is_clover_armed()
  print((pos.pose.position.x, pos.pose.position.y), is_still, armed)
  landed = False
  if is_still and not armed:
    landing['Landed'].set_score(3)
    landed = True
  else:
    landing['Landed'].set_score(0)

  if landed and distance_between_points(
      (pos.pose.position.x, pos.pose.position.y), marker_positions[0]) < 0.4:
    landing['On platform'].set_score(7)
  else:
    landing['On platform'].set_score(0)


task_server = TaskServer()
server_handler = get_servers_handler([task_server], timeout=0.2)

while True:
  server_handler()
  check_landing()
