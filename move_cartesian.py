#!/usr/bin/env python3
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory


class MoveCartesian(Node):
    def __init__(self):
        super().__init__("move_cartesian")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.group = "manipulator"
        self.ee_link = "ee_link"
        self.base_frame = "base_link"
        self.joint_names = [
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint",
        ]

        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        self._js = None

    def get_joint_state(self):
        sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.get_logger().info("Ждём /joint_states...")
        while self._js is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        return self._js

    def _js_cb(self, msg):
        self._js = msg

    def get_current_pose(self, js):
        self.fk_cli.wait_for_service()
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.ee_link]
        req.robot_state.joint_state = js
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        pose = resp.pose_stamped[0].pose
        self.get_logger().info(
            f"Текущая поза TCP: x={pose.position.x:.3f} "
            f"y={pose.position.y:.3f} z={pose.position.z:.3f}"
        )
        return pose

    def move_line(self, dx=0.0, dy=0.0, dz=-0.15):
        js = self.get_joint_state()
        start_pose = self.get_current_pose(js)

        target = copy.deepcopy(start_pose)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        self.cart_cli.wait_for_service()
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.start_state.joint_state = js
        req.group_name = self.group
        req.link_name = self.ee_link
        req.waypoints = [target]      
        req.max_step = 0.01          
        req.jump_threshold = 0.0     
        req.avoid_collisions = True

        self.get_logger().info(f"Считаем прямую на dz={dz}...")
        fut = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()

        fraction = resp.fraction
        self.get_logger().info(f"Построено {fraction*100:.0f}% пути")
        if fraction < 0.5:
            self.get_logger().error("Слишком мало пути построено, отмена.")
            return

        self.exec_cli.wait_for_server()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution
        self.get_logger().info("Едем по прямой...")
        send = self.exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        self.get_logger().info("Готово.")


def main():
    rclpy.init()
    node = MoveCartesian()
    # прямая вниз на 15 см
    node.move_line(dz=-0.15)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()