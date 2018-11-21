import logging
logging.basicConfig(level=logging.DEBUG)
import rospy

# imports the service
from prac2cram.srv import Prac2Cram
# import the messages
from prac2cram.msg import Task, ActionCore, ActionRole, KeyValue

def getROSActionCores(action_cores_RPC):
  action_cores_ROS = []
  for action_core_RPC in action_cores_RPC:
    action_core_ROS = ActionCore()
    action_core_ROS.action_core_name = action_core_RPC['action_core_name']
    action_core_ROS.action_roles = []
    for role_RPC in action_core_RPC['action_roles']:
      role_ROS = ActionRole()
      role_ROS.role_name=role_RPC['role_name']
      if isinstance(role_RPC['role_value'], str):
          role_ROS.role_value=role_RPC['role_value']
      elif isinstance(role_RPC['role_value'], dict):
          role_ROS.role_values = []
          for k, v in role_RPC['role_value'].items():
              role_ROS.role_values.append(KeyValue(key=k, value=v))
      action_core_ROS.action_roles.append(role_ROS)
    action_cores_ROS.append(action_core_ROS)
  return action_cores_ROS

def getROSTasks(tasks_RPC):

  print 'received: %s' %tasks_RPC
  tasks_ROS = []
  for task in tasks_RPC:
    tasks_ROS.append(Task(action_cores = getROSActionCores(task['action_cores'])))
  print 'tasks_ROS: %s' %tasks_ROS
  return tasks_ROS

# for the generated plan strings
def getStringList(strings_ROS):
  strings_RPC = []
  for string in strings_ROS:
    strings_RPC.append(string)
  return strings_RPC

