import rospy
from gazebo_msgs.srv import GetJointProperties, GetModelState
from .utils import distance_between_points

def match_cordinates(correct, prediction):
  idxs = []


  correct_markeable = correct.copy()
  prediction_markeable = prediction.copy()

  def get_closest(a):
    return min(correct_markeable, key=lambda x: distance_between_points(x, a))


  while len(correct_markeable) > 0:
    best_pred = min(prediction_markeable,
                    key=lambda x: distance_between_points(x, get_closest(x)))
    best_correct = get_closest(best_pred)

    idxs.append((correct.index(best_correct), prediction.index(best_pred)))
    correct_markeable.remove(best_correct)
    prediction_markeable.remove(best_pred)

  return idxs

rospy.init_node("cloversim_checker")
rospy.wait_for_service('/gazebo/get_joint_properties')
rospy.wait_for_service('/gazebo/get_model_state')

get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def is_clover_armed():
  props = get_joint_properties('rotor_0_joint')
  return props.rate[0] > 0.1

def get_clover_position():
  props = get_model_state('clover', '')
  return props

def is_clover_still(pos):
  speed = 0
  speed += pos.twist.linear.x
  speed += pos.twist.linear.y
  speed += pos.twist.linear.z
  return speed < 0.05
