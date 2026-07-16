#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
)


class MoveToHome(Node):
    def __init__(self):
        super().__init__("move_to_home")
        self._client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self, joint_positions):
        self.get_logger().info("Ждём move_group...")
        self._client.wait_for_server()

        joint_names = [
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint",
        ]

        constraints = Constraints()
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = "manipulator"
        req.goal_constraints.append(constraints)
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        self.get_logger().info("отправляется цель")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Цель отклонена move_group.")
            return
        self.get_logger().info("Цель принята, планирую и выполняю")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Готово")


def main():
    rclpy.init()
    node = MoveToHome()
    node.send_goal([
    -0.6,
    -0.8,
    1.4,
    -0.3,
    1.57,
    -0.5
    ])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
