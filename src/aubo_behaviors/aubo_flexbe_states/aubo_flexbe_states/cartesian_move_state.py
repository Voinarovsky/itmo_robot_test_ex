#!/usr/bin/env python

import rclpy
import tf2_ros

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory


class CartesianMoveState(EventState):

    SRV = '/compute_cartesian_path'
    ACT = '/execute_trajectory'
    BASE = 'base_link'
    EE = 'ee_link'
    GROUP = 'manipulator'

    def __init__(self, dx=0.0, dy=0.0, dz=0.15,
                 min_fraction=0.9, max_step=0.01, timeout=30.0):
        super().__init__(outcomes=['done', 'failed', 'timeout'])

        self._dx = float(dx)
        self._dy = float(dy)
        self._dz = float(dz)
        self._min_fraction = float(min_fraction)
        self._max_step = float(max_step)
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = float(timeout)

        node = CartesianMoveState._node

        ProxyServiceCaller.initialize(node)
        ProxyActionClient.initialize(node)

        self._caller = ProxyServiceCaller({self.SRV: GetCartesianPath})
        self._client = ProxyActionClient({self.ACT: ExecuteTrajectory},
                                         wait_duration=0.0)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, node, spin_thread=True)

        self._error = False
        self._return = None
        self._start_time = None

    # ------------------------------------------------------------------
    def _lookup(self, tries=20):
        """Текущая поза ee_link относительно base_link."""
        for _ in range(tries):
            try:
                return self._tf_buffer.lookup_transform(
                    self.BASE, self.EE, rclpy.time.Time(),
                    timeout=Duration(seconds=0.5))
            except Exception:  # pylint: disable=W0703
                continue
        return None

    # ------------------------------------------------------------------
    def on_enter(self, userdata):
        self._error = False
        self._return = None
        self._start_time = self._node.get_clock().now()

        tf = self._lookup()
        if tf is None:
            Logger.logwarn(
                f'CartesianMoveState: нет TF {self.BASE} -> {self.EE}')
            self._error = True
            return

        target = Pose()
        target.position.x = tf.transform.translation.x + self._dx
        target.position.y = tf.transform.translation.y + self._dy
        target.position.z = tf.transform.translation.z + self._dz
        target.orientation = tf.transform.rotation

        Logger.loginfo(
            f'Cartesian: ({tf.transform.translation.x:.3f}, '
            f'{tf.transform.translation.y:.3f}, '
            f'{tf.transform.translation.z:.3f}) -> '
            f'({target.position.x:.3f}, {target.position.y:.3f}, '
            f'{target.position.z:.3f})')

        req = GetCartesianPath.Request()
        req.header.frame_id = self.BASE
        req.header.stamp = self._node.get_clock().now().to_msg()
        req.group_name = self.GROUP
        req.link_name = self.EE
        req.waypoints = [target]
        req.max_step = self._max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        try:
            res = self._caller.call(self.SRV, req)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(
                f'Сервис {self.SRV} недоступен:\n  {type(exc)} - {exc}')
            self._error = True
            return

        n_pts = len(res.solution.joint_trajectory.points)
        Logger.loginfo(f'fraction = {res.fraction:.3f}, точек = {n_pts}')

        if res.fraction < self._min_fraction:
            Logger.logwarn(
                f'Линия построена на {res.fraction * 100:.0f}% '
                f'(порог {self._min_fraction * 100:.0f}%) — не еду. '
                'Вероятно сингулярность, лимит сустава или коллизия.')
            self._error = True
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution

        try:
            self._client.send_goal(self.ACT, goal,
                                   wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(
                f'Не удалось отправить траекторию:\n  {type(exc)} - {exc}')
            self._error = True

    # ------------------------------------------------------------------
    def execute(self, userdata):
        if self._error:
            return 'failed'

        if self._return is not None:
            return self._return

        if self._client.has_result(self.ACT):
            result = self._client.get_result(self.ACT)
            code = result.error_code.val
            if code == 1:
                Logger.loginfo('Cartesian motion complete')
                self._return = 'done'
            else:
                Logger.logwarn(f'Исполнение не удалось, error_code = {code}')
                self._return = 'failed'
            return self._return

        elapsed = (self._node.get_clock().now().nanoseconds
                   - self._start_time.nanoseconds)
        if elapsed > self._timeout.nanoseconds:
            Logger.logwarn('CartesianMoveState: timeout')
            self._return = 'timeout'
            return 'timeout'

        return None

    # ------------------------------------------------------------------
    def on_exit(self, userdata):
        if not self._client.has_result(self.ACT):
            self._client.cancel(self.ACT)
            Logger.loginfo('Отменена активная траектория.')