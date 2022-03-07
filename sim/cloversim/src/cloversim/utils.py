import math
import selectors

def distance_between_points(a, b):
  return math.sqrt(sum((aa - bb) **2 for aa, bb in zip(a, b)))

def get_servers_handler(servers, timeout=0.01):

  sel = selectors.DefaultSelector()
  for server in servers:
    sel.register(server, selectors.EVENT_READ)

  def handler():
    events = sel.select(timeout)
    for key, mask in events:
      key.fileobj.handle_request()
  return handler