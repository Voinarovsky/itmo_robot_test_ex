#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionFK, GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import copy
import time


class AuboFlexBEBehavior(Node):

    def __init__(self):
        super().__init__("aubo_flexbe_behavior")
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._cart_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._js = None

        self.joint_names = [
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint",
        ]


    def state_move_joints(self, positions, label="joint_move"):
        self.get_logger().info(f"[STATE: {label}] Планируем joint-space движение...")
        self._move_client.wait_for_server()

        constraints = Constraints()
        for name, pos in zip(self.joint_names, positions):
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

        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f"[STATE: {label}] Цель отклонена!")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"[STATE: {label}] Выполнено!")
        return True


    def state_move_cartesian(self, dx=0.0, dy=0.0, dz=0.0, label="cartesian_move"):
        self.get_logger().info(f"[STATE: {label}] Планируем Cartesian движение dx={dx} dy={dy} dz={dz}...")

        # Получаем текущие углы
        self._js = None
        sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        while self._js is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)

        # FK: текущая поза TCP
        self._fk_client.wait_for_service()
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = "base_link"
        fk_req.fk_link_names = ["ee_link"]
        fk_req.robot_state.joint_state = self._js
        fk_fut = self._fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, fk_fut)
        current_pose = fk_fut.result().pose_stamped[0].pose

        # Целевая точка
        target = copy.deepcopy(current_pose)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        # Cartesian path
        self._cart_client.wait_for_service()
        cart_req = GetCartesianPath.Request()
        cart_req.header.frame_id = "base_link"
        cart_req.start_state.joint_state = self._js
        cart_req.group_name = "manipulator"
        cart_req.link_name = "ee_link"
        cart_req.waypoints = [target]
        cart_req.max_step = 0.01
        cart_req.jump_threshold = 0.0
        cart_req.avoid_collisions = True

        cart_fut = self._cart_client.call_async(cart_req)
        rclpy.spin_until_future_complete(self, cart_fut)
        resp = cart_fut.result()

        self.get_logger().info(f"[STATE: {label}] Построено {resp.fraction*100:.0f}% пути")
        if resp.fraction < 0.5:
            self.get_logger().error(f"[STATE: {label}] Мало пути, отмена.")
            return False

        self._exec_client.wait_for_server()
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = resp.solution
        send = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        self.get_logger().info(f"[STATE: {label}] Выполнено!")
        return True

    def _js_cb(self, msg):
        self._js = msg

    def run_behavior(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("FLEXBE BEHAVIOR: Запуск последовательности движений")
        self.get_logger().info("=" * 50)

        self.get_logger().info("\n>>> TRANSITION: start → move_to_home")
        ok = self.state_move_joints(
            [0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0],
            label="move_to_home"
        )
        if not ok:
            self.get_logger().error("BEHAVIOR OUTCOME: failed")
            return
        time.sleep(1)

        # State 2: Joint-space → поза 1
        self.get_logger().info("\n>>> TRANSITION: move_to_home → move_to_pose1")
        ok = self.state_move_joints(
            [0.4, -0.5, 1.1, -0.7, 1.57, 0.3],
            label="move_to_pose1"
        )
        if not ok:
            self.get_logger().error("BEHAVIOR OUTCOME: failed")
            return
        time.sleep(1)

        self.get_logger().info("\n>>> TRANSITION: move_to_pose1 → cartesian_down")
        ok = self.state_move_cartesian(
            dz=-0.1,
            label="cartesian_down"
        )
        if not ok:
            self.get_logger().error("BEHAVIOR OUTCOME: failed")
            return
        time.sleep(1)

        self.get_logger().info("\n>>> TRANSITION: cartesian_down → cartesian_right")
        ok = self.state_move_cartesian(
            dy=0.1,
            label="cartesian_right"
        )
        if not ok:
            self.get_logger().error("BEHAVIOR OUTCOME: failed")
            return
        time.sleep(1)

        self.get_logger().info("\n>>> TRANSITION: cartesian_right → return_home")
        ok = self.state_move_joints(
            [0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0],
            label="return_home"
        )
        if not ok:
            self.get_logger().error("BEHAVIOR OUTCOME: failed")
            return

        self.get_logger().info("=" * 50)
        self.get_logger().info("BEHAVIOR OUTCOME: finished (все движения выполнены)")
        self.get_logger().info("=" * 50)


def main():
    rclpy.init()
    node = AuboFlexBEBehavior()
    node.run_behavior()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()