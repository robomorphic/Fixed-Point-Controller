import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from time import sleep
from std_msgs.msg import Float32
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import RobotTrajectory
import copy
import json
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from actionlib_msgs.msg import GoalStatusArray, GoalID

GOAL_TOLERANCE = 0.001
PLAN_SCALER = 0.45

position_array = []
velocity_array = []
time_array = []


def print_plan(plan):
    for point in plan.joint_trajectory.points:
        position_array.append(point.positions[:6])
        velocity_array.append(point.velocities[:6])
        time_array.append(point.time_from_start.to_sec())
    print("printing plan:")
    # need to print the arrays in C++ format
    print("double position_array[][6] = {")
    for i in range(len(position_array)):
        print("{", end="")
        for j in range(len(position_array[i])):
            print(str(position_array[i][j]), end="")
            if j < len(position_array[i]) - 1:
                print(", ", end="")
        print("},")
    print("};")
    print("double velocity_array[][6] = {")
    for i in range(len(velocity_array)):
        print("{", end="")
        for j in range(len(velocity_array[i])):
            print(str(velocity_array[i][j]), end="")
            if j < len(velocity_array[i]) - 1:
                print(", ", end="")
        print("},")
    print("};")
    print("double time_array[] = {", end="")
    for i in range(len(time_array)):
        print(str(time_array[i]), end="")
        if i < len(time_array) - 1:
            print(", ", end="")
    print("};")
    #print("position_array: " + str(position_array))
    #print("velocity_array: " + str(velocity_array))
    #print("time_array: " + str(time_array))




rospy.init_node('move_to_start')
rospy.wait_for_message('move_group/status', GoalStatusArray)
commander = MoveGroupCommander('panda_arm')
commander.set_named_target('ready')
commander.set_goal_tolerance(GOAL_TOLERANCE)




initial_position = [-0.002493706342403138, -0.703703218059273, 0.11392999851084838, -2.205860629386432, 0.06983090103997125, 1.5706197776794442]
target_position1 = [0.10167984932684593, -1.18724311909567, -0.09535411201373874, -2.180816145388399, -0.025952730392610403, 1.4625436514020722, 0.7860640899928077]


initial_pose = commander.get_current_pose().pose
initial_pose.position.x = 0.3282550835138732
initial_pose.position.y = 0.05751661222418812
initial_pose.position.z = 0.6330740276406968
initial_pose.orientation.x = -0.9226185998157332
initial_pose.orientation.y = 0.3843317164009613
initial_pose.orientation.z = -0.029732857849977316
initial_pose.orientation.w = 0.013416713696730436

target_pose1 = copy.deepcopy(initial_pose)
target_pose1.position.x = 0.62

target_pose2 = copy.deepcopy(initial_pose)
target_pose2.position.x = 0.62
target_pose2.position.y = 0.2

target_pose3 = copy.deepcopy(initial_pose)
target_pose3.position.y = 0.2

target_pose4 = copy.deepcopy(initial_pose)

poses = [target_pose1, target_pose2, target_pose3, target_pose4]

commander.set_pose_target(initial_pose)
commander.go(wait=True)

plans = []

plan1, _ = commander.compute_cartesian_path(poses, 0.01, 0.0)
plan2 = commander.retime_trajectory(commander.get_current_state(), plan1, PLAN_SCALER)
print_plan(plan2)
plans.append(plan2)
commander.execute(plan2, wait=True)

