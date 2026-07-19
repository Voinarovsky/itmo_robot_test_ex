#!/usr/bin/env python3
import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

GROUP = "manipulator"
BASE = "base_link"
EE = "ee_link"


class CartesianDemo(Node):
    def __init__(self):
        super().__init__("cartesian_demo")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def current_pose(self, timeout=15.0):
        import time
        self.get_logger().info(f"Жду TF {BASE} -> {EE} ...")


        deadline = time.monotonic() + timeout
        last_err = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    BASE, EE, rclpy.time.Time())
            except Exception as e:
                last_err = e
                continue

            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p

        raise RuntimeError(f"TF не пришёл за {timeout} с. Последняя ошибка: {last_err}")

    def compute(self, waypoints):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_cartesian_path не отвечает")

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = GROUP
        req.link_name = EE
        req.waypoints = waypoints
        req.max_step = 0.01        
        req.jump_threshold = 0.0    
        req.avoid_collisions = True

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        self.get_logger().info(f"fraction = {res.fraction:.3f}, "
                               f"точек = {len(res.solution.joint_trajectory.points)}")
        return res

    def execute(self, traj):
        if not self.exec_cli.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory не отвечает")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        fut = self.exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError("Цель отклонена")
        self.get_logger().info("Выполняю...")
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        code = rfut.result().result.error_code.val
        self.get_logger().info(f"error_code = {code} (1 = SUCCESS)")
        return code == 1


def main():
    rclpy.init()
    node = CartesianDemo()

    start = node.current_pose()
    node.get_logger().info(
        f"Старт: x={start.position.x:.3f} y={start.position.y:.3f} z={start.position.z:.3f}")

    wps = []
    p = copy.deepcopy(start)
    p.position.z -= 0.20
    wps.append(p)

    res = node.compute(wps)
    if res.fraction < 0.9:
        node.get_logger().error(
            f"Линия построена только {res.fraction*100:.0f}%. "
            "Не едет:(((")
        rclpy.shutdown()
        sys.exit(1)

    node.execute(res.solution)
    rclpy.shutdown()


if __name__ == "__main__":
    main()