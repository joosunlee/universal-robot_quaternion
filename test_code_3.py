#!/usr/bin/env python


import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf


rospy.init_node('ur5_test', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = robot.get_group_names()
print("%s" %group_name)
print("%s" %robot.get_current_state())

move_group = moveit_commander.MoveGroupCommander(group_name[1])
move_group.set_named_target('up')
plan = move_group.plan()
move_group.execute(plan, wait=True)

def move_joints(move_group, goal):
  move_group.go(goal, wait=True)
  move_group.stop()

def get_joint_state(move_group):
  joint_state = move_group.get_current_joint_values()
  print "--> current joint state as follows (rad) :"
  print joint_state
  print "--> current joint state as follows (degree) :"
  print [joint*180./math.pi for joint in joint_state]

get_joint_state(move_group)
goal = [n*math.pi/180. for n in [90., -100., 90., -100.,-90.,0.]]

move_joints(move_group, goal)
get_joint_state(move_group)

#import geometry_msgs.msg
pose_goal = geometry_msgs.msg.Pose()

def get_goal_pose(move_group):
  joint_state = move_group.get_current_pose()
  print "--> geometry_goal_pose :"
  
  return joint_state

pose_goal = get_goal_pose(move_group)
print pose_goal
print pose_goal.pose.orientation

#tf

quat_angle = tf.transformations.euler_from_quaternion([
  pose_goal.pose.orientation.x,
  pose_goal.pose.orientation.y,
  pose_goal.pose.orientation.z,
  pose_goal.pose.orientation.w,

])

[each*180./math.pi for each in quat_angle]
print [each*180./math.pi for each in quat_angle]

[p, q, r] = [each*math.pi/180. for each in [0.,30.,90.]]

x,y,z,w = tf.transformations.quaternion_from_euler(p,q,r)

pose_goal.pose.orientation.x =x
pose_goal.pose.orientation.y =y
pose_goal.pose.orientation.z =z
pose_goal.pose.orientation.w =w

move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

