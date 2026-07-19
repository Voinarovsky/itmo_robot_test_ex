#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Define TestMove.

Autonomous joint-space motion sequence for the aubo_i7 robot.
home -> pose1 -> pose2 -> home.

@author: kir
"""
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import OperatableStateMachine
from flexbe_core import initialize_flexbe_core
from flexbe_moveit2_flexbe_states.execute_trajectory_state import ExecuteTrajectoryState
from flexbe_moveit2_flexbe_states.move_group_joint_plan_state import MoveGroupJointPlanState


class TestMoveSM(Behavior):
    """Autonomous joint-space motion sequence for aubo_i7."""

    def __init__(self, node):
        super().__init__()
        self.name = 'TestMove'
        self.add_parameter('plan_action', '/move_action')
        self.add_parameter('execute_action', '/execute_trajectory')
        self.add_parameter('planning_time', 5.0)
        initialize_flexbe_core(node)

    def create(self):
        """Create state machine."""
        joints = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                  'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        home = [0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0]
        pose1 = [0.4, -0.5, 1.1, -0.7, 1.57, 0.3]
        pose2 = [-0.6, -0.8, 1.4, -0.3, 1.57, -0.5]

        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.move_group = "manipulator"
        _state_machine.userdata.joint_names = joints
        _state_machine.userdata.joint_values = home
        _state_machine.userdata.planner_id = ""
        _state_machine.userdata.status_text = ""

        _plan_outcomes = {'param_error': Autonomy.Off, 'planning_failed': Autonomy.Off,
                          'planned': Autonomy.Off, 'failed': Autonomy.Off,
                          'unavailable': Autonomy.Off, 'timeout': Autonomy.Off}
        _exec_outcomes = {'done': Autonomy.Off, 'failed': Autonomy.Off,
                          'unavailable': Autonomy.Off, 'param_error': Autonomy.Off}
        _plan_remap = {'move_group': 'move_group', 'joint_names': 'joint_names',
                       'joint_values': 'joint_values', 'planner_id': 'planner_id',
                       'planned_trajectory': 'planned_trajectory',
                       'planning_time': 'planning_time', 'error_msg': 'status_text'}
        _exec_remap = {'trajectory': 'planned_trajectory', 'status_text': 'status_text',
                       'goal_names': 'goal_names', 'goal_values': 'goal_values'}

        with _state_machine:
            OperatableStateMachine.add('PlanHome',
                                       MoveGroupJointPlanState(action_topic=self.plan_action,
                                                               move_group='manipulator',
                                                               joint_names=joints,
                                                               joint_values=home,
                                                               verbose=False,
                                                               joint_tolerance=0.01,
                                                               constraint_weight=1.0,
                                                               allowed_planning_time=self.planning_time,
                                                               planner_id='',
                                                               timeout=60.0),
                                       transitions={'planned': 'ExecHome', 'planning_failed': 'failed',
                                                    'param_error': 'failed', 'failed': 'failed',
                                                    'unavailable': 'failed', 'timeout': 'failed'},
                                       autonomy=_plan_outcomes,
                                       remapping=_plan_remap)

            OperatableStateMachine.add('ExecHome',
                                       ExecuteTrajectoryState(action_topic=self.execute_action,
                                                              verbose=False, timeout=60.0, max_delay=-1.0),
                                       transitions={'done': 'PlanPose1', 'failed': 'failed',
                                                    'unavailable': 'failed', 'param_error': 'failed'},
                                       autonomy=_exec_outcomes,
                                       remapping=_exec_remap)

            OperatableStateMachine.add('PlanPose1',
                                       MoveGroupJointPlanState(action_topic=self.plan_action,
                                                               move_group='manipulator',
                                                               joint_names=joints,
                                                               joint_values=pose1,
                                                               verbose=False,
                                                               joint_tolerance=0.01,
                                                               constraint_weight=1.0,
                                                               allowed_planning_time=self.planning_time,
                                                               planner_id='',
                                                               timeout=60.0),
                                       transitions={'planned': 'ExecPose1', 'planning_failed': 'failed',
                                                    'param_error': 'failed', 'failed': 'failed',
                                                    'unavailable': 'failed', 'timeout': 'failed'},
                                       autonomy=_plan_outcomes,
                                       remapping=_plan_remap)

            OperatableStateMachine.add('ExecPose1',
                                       ExecuteTrajectoryState(action_topic=self.execute_action,
                                                              verbose=False, timeout=60.0, max_delay=-1.0),
                                       transitions={'done': 'PlanPose2', 'failed': 'failed',
                                                    'unavailable': 'failed', 'param_error': 'failed'},
                                       autonomy=_exec_outcomes,
                                       remapping=_exec_remap)

            OperatableStateMachine.add('PlanPose2',
                                       MoveGroupJointPlanState(action_topic=self.plan_action,
                                                               move_group='manipulator',
                                                               joint_names=joints,
                                                               joint_values=pose2,
                                                               verbose=False,
                                                               joint_tolerance=0.01,
                                                               constraint_weight=1.0,
                                                               allowed_planning_time=self.planning_time,
                                                               planner_id='',
                                                               timeout=60.0),
                                       transitions={'planned': 'ExecPose2', 'planning_failed': 'failed',
                                                    'param_error': 'failed', 'failed': 'failed',
                                                    'unavailable': 'failed', 'timeout': 'failed'},
                                       autonomy=_plan_outcomes,
                                       remapping=_plan_remap)

            OperatableStateMachine.add('ExecPose2',
                                       ExecuteTrajectoryState(action_topic=self.execute_action,
                                                              verbose=False, timeout=60.0, max_delay=-1.0),
                                       transitions={'done': 'PlanReturnHome', 'failed': 'failed',
                                                    'unavailable': 'failed', 'param_error': 'failed'},
                                       autonomy=_exec_outcomes,
                                       remapping=_exec_remap)

            OperatableStateMachine.add('PlanReturnHome',
                                       MoveGroupJointPlanState(action_topic=self.plan_action,
                                                               move_group='manipulator',
                                                               joint_names=joints,
                                                               joint_values=home,
                                                               verbose=False,
                                                               joint_tolerance=0.01,
                                                               constraint_weight=1.0,
                                                               allowed_planning_time=self.planning_time,
                                                               planner_id='',
                                                               timeout=60.0),
                                       transitions={'planned': 'ExecReturnHome', 'planning_failed': 'failed',
                                                    'param_error': 'failed', 'failed': 'failed',
                                                    'unavailable': 'failed', 'timeout': 'failed'},
                                       autonomy=_plan_outcomes,
                                       remapping=_plan_remap)

            OperatableStateMachine.add('ExecReturnHome',
                                       ExecuteTrajectoryState(action_topic=self.execute_action,
                                                              verbose=False, timeout=60.0, max_delay=-1.0),
                                       transitions={'done': 'finished', 'failed': 'failed',
                                                    'unavailable': 'failed', 'param_error': 'failed'},
                                       autonomy=_exec_outcomes,
                                       remapping=_exec_remap)

        return _state_machine