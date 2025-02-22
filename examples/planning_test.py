import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

class ExamplePlanningNode(Node):
    def __init__(self):
        super().__init__('basic_planning_node')
        self.trajectory_publisher = self.create_publisher(JointTrajectory, 'display_planned_path', 10)

        # Subscribe to current joint state
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None

        # Create a service client for the /plan_kinematic_path service
        self.plan_kinematic_path_client = self.create_client(GetMotionPlan, '/plan_kinematic_path_cfs')

        # Wait for the service to be available
        while not self.plan_kinematic_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create an action client for the /execute_trajectory action server
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def joint_state_callback(self, joint_state):
        self.current_joint_state = joint_state

    def plan_kinematic_path(self):
        # Obtain the current joint state
        while self.current_joint_state is None:
            self.get_logger().info('Waiting for joint state...')
            rclpy.spin_once(self)
        
        self.get_logger().info(f'Current joint state: {self.current_joint_state}')

        # Create a request
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = "ur_manipulator"  # Replace with your group name
        request.motion_plan_request.start_state.joint_state.name = self.current_joint_state.name
        request.motion_plan_request.start_state.joint_state.position = self.current_joint_state.position

        # Define the goal constraints
        goal_constraints = Constraints()

        # # try joint constraints
        # joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        # joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # for joint_name, joint_position in zip(joint_names, joint_positions):
        #     joint_constraint = JointConstraint()
        #     joint_constraint.joint_name = joint_name
        #     joint_constraint.position = joint_position
        #     goal_constraints.joint_constraints.append(joint_constraint)

        # try position constraints
        position_constraint = PositionConstraint()
        position_constraint.link_name = "tool0"
        position_constraint.header.frame_id = "base_link"
        position_constraint.target_point_offset.x = -0.11283
        position_constraint.target_point_offset.y = -0.55518
        position_constraint.target_point_offset.z = 0.52482
        position_constraint.constraint_region.primitives = [
            SolidPrimitive()
        ]
        position_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        position_constraint.constraint_region.primitives[0].dimensions = [0.01, 0.01, 0.01]
        position_constraint.constraint_region.primitive_poses = [
            Pose()
        ]
        goal_constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "tool0"
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        goal_constraints.orientation_constraints.append(orientation_constraint)

        request.motion_plan_request.goal_constraints.append(goal_constraints)

        # Send the request
        future = self.plan_kinematic_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Plan received')
            # Process the result here
            return future.result()
        else:
            self.get_logger().error('Failed to call service /plan_kinematic_path')
            return None

    def execute_trajectory(self, trajectory):
        # Create a goal message
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        # Send the goal
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('Sending goal to execute trajectory')
        future = self.execute_trajectory_client.send_goal_async(goal_msg)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result is not None:
            self.get_logger().info('Trajectory execution completed')
        else:
            self.get_logger().error('Failed to execute trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = ExamplePlanningNode()
    result = node.plan_kinematic_path()
    if result is not None:
        # print each position in the result
        for position in result.motion_plan_response.trajectory.joint_trajectory.points:
            print(position.positions)

        # publish the trajectory
        node.trajectory_publisher.publish(result.motion_plan_response.trajectory.joint_trajectory)

        # execute the trajectory
        node.execute_trajectory(result.motion_plan_response.trajectory)
    else:
        print("Failed to get a valid plan")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
