from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import moveit
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import BoundingVolume
from shape_msgs.msg import SolidPrimitive

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import xarm7

from trajectory_msgs.msg import JointTrajectoryPoint
print("all libs loaded!")
class XarmMotionPlanner(Node):
    def __init__(self):
        super().__init__('xarm_motion_planner')

        # Initialize the action client for MoveGroup
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()


    def move_to_joint_state(self, joint_goal):
        # Create a MoveGroup action goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "xarm7"
        
        # Define joint constraints for target position
        joint_constraints = []
        for i, joint_value in enumerate(joint_goal):
            constraint = JointConstraint()
            constraint.joint_name = f"joint{i+1}"
            constraint.position = float(joint_value)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            joint_constraints.append(constraint)
        
        goal.request.goal_constraints.append(Constraints(joint_constraints=joint_constraints))
        
        # Send goal to the MoveGroup action server
        self.get_logger().info("Sending goal to MoveGroup action server...")
        self._action_client.send_goal_async(goal)

    def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
        else:
                plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
        else:
                logger.error("Planning failed")

    def execute_trajectory(self, joint_trajectory):
        goal = MoveGroup.Goal()
        goal.trajectory.joint_trajectory.points = joint_trajectory.points
        self._action_client.send_goal_async(goal)

def main():
    #rclpy.init()
    #planner = XarmMotionPlanner()
    #logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    # Example: Move to a specific joint state
    #joint_goal = [0, -0.5, 0, -1.0, 0, 1.0, 0]
    #joint_goal = [0,0 , 0, 0, 0, 0, 0]
    #planner.move_to_joint_state(joint_goal)

    # Example: Move to a specific pose (PoseStamped)
    # target_pose = PoseStamped()
    # target_pose.header.frame_id = "link_base"  # Set to the correct frame
    # target_pose.header.stamp = rclpy.time.Time().to_msg()
    # target_pose.pose.position.x = 0.5
    # target_pose.pose.position.y = 0.0
    # target_pose.pose.position.z = 0.5
    # target_pose.pose.orientation.x = 0.0
    # target_pose.pose.orientation.y = 0.0
    # target_pose.pose.orientation.z = 0.0
    # target_pose.pose.orientation.w = 1.0
    # planner.move_to_pose_goal(target_pose)

    #xarm = MoveItPy(node_name="moveit_py")
    #xarm_arm = xarm.panda.get_planning_component("xarm")

    # set plan start state to current state
    # xarm_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = "link_base"
    # pose_goal.pose.orientation.w = 1.0
    # pose_goal.pose.position.x = 0.28
    # pose_goal.pose.position.y = -0.2
    # pose_goal.pose.position.z = 0.5
    # xarm_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_eef")

    # plan to goal
    # planner.plan_and_execute(xarm, xarm_arm, logger)

    #rclpy.spin(planner)
    #planner.destroy_node()
    #rclpy.shutdown()
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.2, 0.2, 0.0])
    node.declare_parameter("quat_xyzw", [-1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("synchronous", True)
    # If non-positive, don't cancel. Only used if synchronous is False
    node.declare_parameter("cancel_after_secs", 0.0)
    # Planner ID
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    # Declare parameters for cartesian planning
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=xarm7.joint_names(),
        base_link_name=xarm7.base_link_name() ,
        end_effector_name=xarm7.end_effector_name(),
        group_name=xarm7.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.1 #0.5
    moveit2.max_acceleration = 0.1 # 0.5

    #test get current state
    
    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = (
        node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    )
    cartesian_fraction_threshold = (
        node.get_parameter("cartesian_fraction_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_jump_threshold = (
        node.get_parameter("cartesian_jump_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_avoid_collisions = (
        node.get_parameter("cartesian_avoid_collisions")
        .get_parameter_value()
        .bool_value
    )

    # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=cartesian,
        cartesian_max_step=cartesian_max_step,
        cartesian_fraction_threshold=cartesian_fraction_threshold,
    )
    if synchronous:
        # Note: the same functionality can be achieved by setting
        # `synchronous:=false` and `cancel_after_secs` to a negative value.
        moveit2.wait_until_executed()
    else:
        # Wait for the request to get accepted (i.e., for execution to start)
        print("Current State: " + str(moveit2.query_state()))
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        # Get the future
        print("Current State: " + str(moveit2.query_state()))
        future = moveit2.get_execution_future()

        # Cancel the goal
        if cancel_after_secs > 0.0:
            # Sleep for the specified time
            sleep_time = node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            # Cancel the goal
            print("Cancelling goal")
            moveit2.cancel_execution()

        # Wait until the future is done
        while not future.done():
            rate.sleep()

        # Print the result
        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == '__main__':
    main()
