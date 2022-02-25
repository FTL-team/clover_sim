import rospy
import os

TASK_PATH = "/tmp/task_generated"
WORLD_PATH = TASK_PATH + "/task.world"

if not os.path.exists(TASK_PATH):
  os.mkdir(TASK_PATH)
  os.mkdir(TASK_PATH + "/models")