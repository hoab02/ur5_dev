#!/usr/bin/env python3
import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = ["joint_trajectory_controller", 
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "my_cartesian_motion_controller",
    "my_cartesian_force_controller",
    "my_cartesian_compliance_controller",
]

class TrajectoryClient:

    def __init__(self):
        rospy.init_node("test_move")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[2]

    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        # position_list = [[0.7853, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]] #home
        position_list = [[-0.9470, -1.5079, -1.6336, -1.5528, 1.5915, 0.0]] #home

        duration_list = [3.0]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def interpolate_poses(self, start_pose, end_pose, num_steps):
    # Perform linear interpolation between start and end poses
        a = [start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z] # a,b belong to set type
        b = [end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z]
        c = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z, start_pose.pose.orientation.w]
        d = [end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w]
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

    def move_to_pose(self, my_desired_pose):
        # self.switch_controller(self.cartesian_trajectory_controller)
        pub = rospy.Publisher('/target_frame', PoseStamped, queue_size=10)
        
        # subcribe_data()
        current_pose = rospy.wait_for_message("/current_pose",PoseStamped)
        
        # Create a PoseStamped message for the desired pose
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = "base_link"
        desired_pose.pose.position.x = my_desired_pose[0]
        desired_pose.pose.position.y = my_desired_pose[1]
        desired_pose.pose.position.z = my_desired_pose[2]
        desired_pose.pose.orientation = current_pose.pose.orientation

        num_steps = 100
        interpolated_poses = self.interpolate_poses(current_pose, desired_pose, num_steps)

        rate = rospy.Rate(10)
        for pose in interpolated_poses:
            pub.publish(pose)
            rate.sleep()
            
    def choose_controller(self):
        """Ask the user to select the desired controller from the available list."""
        rospy.loginfo("Available trajectory controllers:")
        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (joint-based): {}".format(index, name))
        for (index, name) in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (Cartesian): {}".format(index + len(JOINT_TRAJECTORY_CONTROLLERS), name))
        choice = -1
        while choice < 0:
            input_str = input(
                "Please choose a controller by entering its number (Enter '0' if "
                "you are unsure / don't care): "
            )
            try:
                choice = int(input_str)
                if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(
                    CARTESIAN_TRAJECTORY_CONTROLLERS
                ):
                    rospy.loginfo(
                        "{} not inside the list of options. "
                        "Please enter a valid index from the list above.".format(choice)
                    )
                    choice = -1
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")
        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
            return "joint_based"

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[
            choice - len(JOINT_TRAJECTORY_CONTROLLERS)
        ]
        return "cartesian"

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)


if __name__ == "__main__":
    
    lo_be_y = (243.85036554171813 / 1000.0)
    lo_be_x = (-82.3215711518279 / 1000.0)
    tr_be_y = (114.85036554171812 / 1000.0)
    tr_be_x = (-59.321571151827904 /1000.0) 
    lo_to_y = (253.85036554171813 / 1000.0)
    lo_to_x = (37.678428848172096 / 1000.0)
    tr_to_y = (122.85036554171812/ 1000.0)
    tr_to_x = (40.678428848172096/ 1000.0)

    client = TrajectoryClient()
    client.choose_controller()
    client.send_joint_trajectory() #move robot to home position
    # client.switch_controller("my_cartesian_motion_controller")
    
    # client.move_to_pose([0.406 - lo_to_x -0.005, 0.210 - lo_to_y + 0.01  , 0.388])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - lo_to_x -0.005, 0.210 - lo_to_y + 0.01, 0.10])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - lo_to_x -0.005, 0.210 - lo_to_y + 0.01  , 0.388])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - tr_to_x + 0.015, 0.210 - tr_to_y + 0.01 , 0.388])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - tr_to_x + 0.015, 0.210 - tr_to_y + 0.01  , 0.10])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - tr_to_x + 0.015, 0.210 - tr_to_y + 0.01   , 0.388])

    # client.move_to_pose([0.406 - lo_be_x -0.005, 0.210 - lo_be_y -0.005  , 0.388])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - lo_be_x -0.005, 0.210 - lo_be_y-0.005 , 0.10])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - lo_be_x -0.005, 0.210 - lo_be_y -0.005 , 0.388])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - tr_be_x +0.01, 0.210 - tr_be_y -0.005  , 0.388])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - tr_be_x +0.01, 0.210 - tr_be_y-0.005 , 0.10])
    # rospy.sleep(5.)
    # client.move_to_pose([0.406 - tr_be_x +0.01 , 0.210 - tr_be_y -0.005 , 0.388])
    # rospy.sleep(5.)
    # client.switch_controller("joint_trajectory_controller")
    # client.send_joint_trajectory()

