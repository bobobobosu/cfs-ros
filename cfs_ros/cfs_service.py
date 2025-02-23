# add current path to sys.path
import sys
import os

sys.path.append(os.getcwd())

from juliacall import Main as jl
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import tempfile

import re
import os
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError
)

os.environ["JULIA_NUM_THREADS"] = "1"
jl.include("julia/CFSRos.jl")

def plan(jl_setup, motion_plan_request):
    # check goal type
    if len(motion_plan_request.goal_constraints) == 0:
        return None
    elif len(motion_plan_request.goal_constraints) > 1:
        # not supported
        return None
    else:
        start_joint_names = np.array(motion_plan_request.start_state.joint_state.name)
        start_joint_positions = np.array(
            motion_plan_request.start_state.joint_state.position
        )
        goal_constraint = motion_plan_request.goal_constraints[0]
        if len(goal_constraint.joint_constraints) > 0:
            # number of joint constraints should be equal to length of start state joint names
            if len(goal_constraint.joint_constraints) != len(
                motion_plan_request.start_state.joint_state.name
            ):
                return None
            goal_joint_names = np.array(
                [x.joint_name for x in goal_constraint.joint_constraints]
            )
            goal_joint_positions = np.array(
                [x.position for x in goal_constraint.joint_constraints]
            )
            return jl.CFSRos.plan_motion_joint(
                jl_setup,
                start_joint_names,
                start_joint_positions,
                goal_joint_names,
                goal_joint_positions,
            )
        else:
            if not (
                len(goal_constraint.position_constraints) == 1
                and len(goal_constraint.orientation_constraints) == 1
            ):
                # not supported
                return None
            pos_constraint = goal_constraint.position_constraints[0]
            ori_constraint = goal_constraint.orientation_constraints[0]

            if pos_constraint.link_name != ori_constraint.link_name:
                # not supported
                return None

            frame_id = pos_constraint.header.frame_id
            goal_link_name = str(pos_constraint.link_name)
            goal_pos = pos_constraint.target_point_offset
            goal_pos = np.array([goal_pos.x, goal_pos.y, goal_pos.z])
            goal_ori = ori_constraint.orientation
            goal_ori = np.array([goal_ori.w, goal_ori.x, goal_ori.y, goal_ori.z])
            return jl.CFSRos.plan_motion_cart(
                jl_setup,
                start_joint_names,
                start_joint_positions,
                frame_id,
                goal_link_name,
                goal_pos,
                goal_ori,
            )


class CFSClient(Node):
    def __init__(self):
        super().__init__("cfs_client")
        self.robot_state_publisher_client = self.create_client(
            GetParameters, "/robot_state_publisher/get_parameters"
        )
        while not self.robot_state_publisher_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "robot_state_publisher service not available, waiting again..."
            )
        self.get_logger().info("robot_state_publisher service available")

        # Create a service for motion planning requests
        self.srv = self.create_service(
            GetMotionPlan, "/plan_kinematic_path_cfs", self.plan_callback
        )
        self.get_logger().info("MotionPlanRequest server is ready.")

        self.jl_setup = jl.CFSRos.robotsetup_setup(self.get_urdf())
        self.get_logger().info("CFSClient is ready.")

    def get_urdf(self):
        self.request = GetParameters.Request()
        self.request.names.append("robot_description")
        self.future = self.robot_state_publisher_client.call_async(self.request)
        # wait for the future to be done
        rclpy.spin_until_future_complete(self, self.future)
        params = self.future.result()
        urdf_string = params.values[0].string_value
        pattern = r'package://([^/]+)(.*)'
        def replacer(match):
            pkg_name = match.group(1)
            relative_path = match.group(2)  # includes the leading '/'
            try:
                pkg_share = get_package_share_directory(pkg_name)
                return os.path.join(pkg_share, relative_path.lstrip('/'))
            except PackageNotFoundError:
                # If not found, just return the original string or handle it
                self.get_logger().warn(f"Package '{pkg_name}' not found; leaving reference unchanged.")
                return match.group(0)  # 'package://pkg_name/...'
        urdf_string = re.sub(pattern, replacer, urdf_string)
        # save urdf_string to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False)
        with open(temp_file.name, 'w') as f:
            f.write(urdf_string)
        return temp_file.name

    def plan_callback(self, request, response):
        """Handles motion planning requests"""
        self.get_logger().info(
            f"Received planning request for {request.motion_plan_request.group_name}"
        )

        traj = plan(self.jl_setup, request.motion_plan_request)
        if traj is None:
            response.motion_plan_response.error_code.val = 2
            return response
        else:
            ts, trajj = traj
            # Assume a basic motion plan with a simple interpolation
            planned_trajectory = JointTrajectory()
            planned_trajectory.joint_names = request.motion_plan_request.start_state.joint_state.name

            for (i, t) in enumerate(ts):
                pt = JointTrajectoryPoint()
                pt.positions = trajj[:, i]
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
                planned_trajectory.points.append(pt)

            # Fill response
            response.motion_plan_response.trajectory.joint_trajectory = (
                planned_trajectory
            )
            response.motion_plan_response.error_code.val = 1  # Success

            self.get_logger().info("Motion plan generated successfully.")
            return response


def run_node(args=None):
    rclpy.init(args=args)
    cfs_client = CFSClient()
    rclpy.spin(cfs_client)
    cfs_client.destroy_node()
    rclpy.shutdown()
