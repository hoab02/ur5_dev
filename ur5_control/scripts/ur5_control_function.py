#!/usr/bin/env python3

import threading
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import LoadController, LoadControllerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np
from std_msgs.msg import String

JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = ["joint_trajectory_controller",
                                ]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "my_cartesian_motion_controller",
    "my_cartesian_force_controller",
    "my_cartesian_compliance_controller",
]

threshold_up = -3
is_robot_moving = False
current_pose_assem = PoseStamped()

def switch_controllers(target_controllers):
    rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.wait_for_service("controller_manager/load_controller")
    other_controllers = (JOINT_TRAJECTORY_CONTROLLERS +
                         CARTESIAN_TRAJECTORY_CONTROLLERS)
    other_controllers.remove(target_controllers)
    switch_controller = rospy.ServiceProxy(
        '/controller_manager/switch_controller', SwitchController)
    load_controller = rospy.ServiceProxy(
        "controller_manager/load_controller", LoadController)
    srv1 = LoadControllerRequest()
    srv1.name = target_controllers
    load_controller(srv1)
    request = SwitchControllerRequest()
    request.stop_controllers = other_controllers
    request.start_controllers = [target_controllers]
    request.strictness = SwitchControllerRequest.BEST_EFFORT
    try:
        response = switch_controller(request)
        if response.ok:
            rospy.loginfo("Controllers switched successfully.")
        else:
            rospy.logwarn("Failed to switch controllers.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

def move_robot_to_home(desired_joint: JointTrajectoryPoint):
    switch_controllers(JOINT_TRAJECTORY_CONTROLLERS[0])
    pub = rospy.Publisher(
        "/joint_trajectory_controller/command", JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Allow time for the publisher to set up
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    desired_joint.time_from_start = rospy.Duration(3.0)
    traj.points.append(desired_joint)
    pub.publish(traj)
    rospy.sleep(2)
    rospy.loginfo("Move robot to home: SUCCEED!")

def interpolate_poses(start_pose, end_pose, num_steps):
    # Perform linear interpolation between start and end poses
    a = [start_pose.pose.position.x, start_pose.pose.position.y,
         start_pose.pose.position.z]  # a,b belong to set type
    b = [end_pose.pose.position.x,
         end_pose.pose.position.y, end_pose.pose.position.z]
    c = [start_pose.pose.orientation.x, start_pose.pose.orientation.y,
         start_pose.pose.orientation.z, start_pose.pose.orientation.w]
    d = [end_pose.pose.orientation.x, end_pose.pose.orientation.y,
         end_pose.pose.orientation.z, end_pose.pose.orientation.w]
    positions = np.linspace(a, b, num_steps)
    orientations = np.linspace(c, d, num_steps)
    interpolated_poses = []
    for i in range(num_steps):
        pose = PoseStamped()
        pose.header.frame_id = start_pose.header.frame_id
        pose.pose.position.x = positions[i][0]
        pose.pose.position.y = positions[i][1]
        pose.pose.position.z = positions[i][2]
        pose.pose.orientation.x = orientations[i][0]
        pose.pose.orientation.y = orientations[i][1]
        pose.pose.orientation.z = orientations[i][2]
        pose.pose.orientation.w = orientations[i][3]
        interpolated_poses.append(pose)
    return interpolated_poses

def move_robot_to_pose(my_desired_pose):
    switch_controllers(CARTESIAN_TRAJECTORY_CONTROLLERS[2])
    pub = rospy.Publisher('/target_frame', PoseStamped, queue_size=10)
    current_pose = rospy.wait_for_message("/current_pose", PoseStamped)
    # Create a PoseStamped message for the desired pose
    desired_pose = PoseStamped()
    desired_pose.header.frame_id = "base_link"
    desired_pose.pose.position.x = my_desired_pose[0]
    desired_pose.pose.position.y = my_desired_pose[1]
    desired_pose.pose.position.z = my_desired_pose[2]
    desired_pose.pose.orientation = current_pose.pose.orientation
    num_steps = 100  # Number of intermediate steps
    interpolated_poses = interpolate_poses(
        current_pose, desired_pose, num_steps)
    for pose in interpolated_poses:
        pub.publish(pose)
        rospy.sleep(0.1)
    rospy.loginfo("Desired pose success")

def gripper_commands(a):  # using only 'o' and 'c' character
    pub = rospy.Publisher('/gripper_command', String, queue_size=10)
    rospy.sleep(1)
    msg = String()
    msg.data = a
    pub.publish(msg)

def sensor_data_callback(msg: PoseStamped):
    global current_pose_assem
    current_pose_assem = msg

### Wrench    
    # global is_robot_moving, threshold
    # is_robot_moving = False
    # if msg.wrench.force.z < threshold_up:
    #     is_robot_moving = True
    # if is_robot_moving:
    #     print("ko an khop")
    # else:
    #     print("an khop")

def thread_funct1():
    sub = rospy.Subscriber("/current_pose", PoseStamped, sensor_data_callback)
    rospy.spin()

def call_back_joint_state():
    current_joint_states = rospy.wait_for_message("/joint_states", JointState)
    return current_joint_states

if __name__ == '__main__':
    
    PRE_ASS = [0.45378456215920726, 0.16329139678717705, 0.2593223728936405]
    GOAL_ASS = [0.45378456215920726, 0.16329139678717705, 0.1972110620300292]

    rospy.init_node('controller_node')
    rate = rospy.Rate(100)

    thread_1 = threading.Thread(target=thread_funct1)
    thread_1.start()
    home = JointTrajectoryPoint()
    home_position = [2.1453502813922327, -1.5522724383375426, 0.3085119426250458, -2.1810008488097132, 4.704324245452881, 3.2166907787323]
    home.positions = home_position

    move_robot_to_home(home)
    rospy.sleep(3)

    move_robot_to_pose(PRE_ASS)  # custom
    rospy.sleep(20)
    print("Done1")
    move_robot_to_pose(GOAL_ASS)  # custom
    rospy.sleep(8)
    print("Done2")

    while not (abs(current_pose_assem.pose.position.z -  GOAL_ASS[2])<=0.035):
        move_robot_to_pose(GOAL_ASS)  # custom
        rospy.sleep(3)

    if not (abs(current_pose_assem.pose.position.z -  GOAL_ASS[2])<=0.005):
        while not (abs(current_pose_assem.pose.position.z -  GOAL_ASS[2])<=0.005):
            assembly_step = JointTrajectoryPoint()
            joints = call_back_joint_state()
            joints : JointState
            a = []
            for temp in joints.position:
                a.append(temp)
            a[5] = a[5] + np.pi/6
            assembly_step.positions = a
            move_robot_to_home(assembly_step)
            rospy.sleep(4.5)
            move_robot_to_pose(GOAL_ASS)  # custom
            rospy.sleep(4.5)
            
    gripper_commands('o')
    rospy.sleep(3)
    move_robot_to_pose(PRE_ASS)  # custom
    rospy.sleep(8)
    move_robot_to_home(home)
    thread_1.join()

    #     assembly_step = JointTrajectoryPoint()
    #     joints = call_back_joint_state()
    #     joints : JointState
    #     a = []
    #     for temp in joints.position:
    #         a.append(temp)
    #     a[5] = a[5] + 2
    #     assembly_step.positions = a
    #     move_robot_to_home(assembly_step)
    #     rospy.sleep(3)
    #     move_robot_to_pose([0.232, 0.248, 0.180])
    #     rospy.sleep(3)
    #     while is_robot_moving:
    #         assembly_step = JointTrajectoryPoint()
    #         joints = call_back_joint_state()
    #         joints : JointState
    #         a = []
    #         for temp in joints.position:
    #             a.append(temp)
    #         a[5] = a[5] + 2
    #         assembly_step.positions = a
    #         move_robot_to_home(assembly_step)
    #         rospy.sleep(3)
    #         move_robot_to_pose([0.232, 0.248, 0.180])
    #         rospy.sleep(3)
    #     break

    # gripper_commands('o')
    # rospy.sleep(3)
    # move_robot_to_pose([0.232, 0.248, 0.244])
    # rospy.sleep(3)
    # move_robot_to_home(home)
    # rospy.sleep(3)
