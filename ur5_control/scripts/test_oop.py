#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import LoadController, LoadControllerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np
from std_msgs.msg import String

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
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

HOME_POSE = [0.5026, -1.5079, 1.5079, -1.5079, -1.5079, 0.0]


class NodeControl():
    def __init__(self):
        rospy.init_node("Controller_Node")
        self.switch_controller_client = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController)
        self.load_controller_client = rospy.ServiceProxy(
            "/controller_manager/load_controller", LoadController)
        self.joint_traj_pub = rospy.Publisher(
            "/joint_trajectory_controller/command", JointTrajectory, queue_size=10)
        self.cartesian_pub = rospy.Publisher(
            '/target_frame', PoseStamped, queue_size=10)
        
        self.load_all_controllers()

    def run(self):
        self.go_to_home_pose()
        rospy.sleep(5)
        # self.set_ee_pose_cartesian([0.385, 0.363, 0.281])
        # rospy.sleep(5000)

    def go_to_home_pose(self):
        self.switch_controller(JOINT_TRAJECTORY_CONTROLLERS[0])
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.joint_names = JOINT_NAMES
        home_pose = JointTrajectoryPoint()
        home_pose.positions = HOME_POSE
        home_pose.time_from_start = rospy.Duration(3.0)
        joint_traj_msg.points.append(home_pose)
        self.joint_traj_pub.publish(joint_traj_msg)

    def set_ee_pose_cartesian(self, desired_pose_list: list):
        self.switch_controller(CARTESIAN_TRAJECTORY_CONTROLLERS[2])
        current_pose: PoseStamped
        current_pose = rospy.wait_for_message("/current_pose", PoseStamped)
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = "base_link"
        
        if desired_pose_list[0] - current_pose.pose.position.x:
            delta_x = (desired_pose_list[0]-current_pose.pose.position.x)
        else:
            delta_x = 0
        if desired_pose_list[1] - current_pose.pose.position.y:
            delta_y = (desired_pose_list[1]-current_pose.pose.position.y)
        else:
            delta_y = 0
        if desired_pose_list[2] - current_pose.pose.position.z:
            delta_z = (desired_pose_list[2]-current_pose.pose.position.z)
        else:
            delta_z = 0
        num_steps = 100

        desired_pose.pose = current_pose.pose
        for i in range(100):
            desired_pose.pose.position.x += i*delta_x/num_steps
            desired_pose.pose.position.y += i*delta_y/num_steps
            desired_pose.pose.position.z += i*delta_z/num_steps
            self.cartesian_pub.publish(desired_pose)

        rospy.loginfo("Desired pose success")

    def load_all_controllers(self):
        rospy.wait_for_service("/controller_manager/load_controller")
        req = LoadControllerRequest()
        for controller in JOINT_TRAJECTORY_CONTROLLERS + CARTESIAN_TRAJECTORY_CONTROLLERS:
            req.name = controller
            self.load_controller_client(req)

    def switch_controller(self, desired_controller):
        rospy.wait_for_service("/controller_manager/switch_controller")
        stopped_controllers = JOINT_TRAJECTORY_CONTROLLERS + \
            CARTESIAN_TRAJECTORY_CONTROLLERS
        stopped_controllers.remove(desired_controller)
        request = SwitchControllerRequest()
        request.stop_controllers = stopped_controllers
        request.start_controllers = [desired_controller]
        request.strictness = SwitchControllerRequest.BEST_EFFORT
        try:
            response = self.switch_controller_client(request)
            if response.ok:
                rospy.loginfo("Controllers switched successfully.")
            else:
                rospy.logwarn("Failed to switch controllers.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))


def main():
    Node = NodeControl()
    Node.run()


if __name__ == "__main__":
    main()
