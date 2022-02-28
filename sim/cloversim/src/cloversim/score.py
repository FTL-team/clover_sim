import json
import enum

class ScoreTask:
  failed = False
  score = 0

  def __init__(self, name, max_score):
    self.name = name
    self.max_score = max_score
    self.parnet= None

  def mark_failed(self):
    self.failed = True
    self.update()

  def set_score(self, score):
    self.score = score
    self.update()

  def update(self):
    if self.parent is None:
      return
    self.parent.update()

  def set_parent(self, parent):
    self.parent = parent

  def as_dict(self):
    return {
      'name': self.name,
      'max_score': self.max_score,
      'score': self.score,
      'failed': self.failed,
      'children': []
    }

class FailMode(enum.Enum):
  ATLEAST_ONE = 0
  ALL = 1
  IGNORE = 2


def create_update(fail_mode=FailMode.ATLEAST_ONE):

  def update(group, tasks):
    tasks_failed = sum(t.failed for t in tasks)
    scor = sum(t.score for t in tasks)
    group.score = scor
    if fail_mode == FailMode.ATLEAST_ONE:
      group.failed = tasks_failed >= 1
    elif fail_mode == FailMode.ALL:
      group.failed = tasks_failed == len(tasks) 
  return update



class ScoreGroup:
  failed = False
  score = 0

  def __init__(self, name, tasks, update_function=create_update()):
    self.name = name
    self.tasks = tasks
    self.max_score = 0
    self.parent = None
    self.update_function = update_function

    for task in tasks:
      task.set_parent(self)
      if task.max_score >= 0:
        self.max_score += task.max_score

  def update(self):
    if self.parent is None:
      return
    self.update_function(self, self.tasks)
    self.parent.update()

  def set_parent(self, parent):
    self.parent = parent
    self.update()

  def as_dict(self):
    return {
        'name': self.name,
        'max_score': self.max_score,
        'score': self.score,
        'failed': self.failed,
        'children': [
          task.as_dict() for task in self.tasks
        ]
    }

  def __getitem__(self, key):
    if type(key) is int:
      return self.tasks[key]
    else:
      for t in self.tasks:
        if t.name == key:
          return t
      return None


class Scoring(ScoreGroup):

  def __init__(self, name, tasks, update_function=create_update(True)):
    super().__init__(name, tasks, update_function)

  def update(self):
    self.update_function(self, self.tasks)
    with open("/home/clover/task_score.json", "w") as f:
      json.dump(self.as_dict(), f)