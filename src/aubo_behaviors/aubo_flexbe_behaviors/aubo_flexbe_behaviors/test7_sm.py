#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Kir
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.

#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define test7.

test

Created on Sun Jul 19 2026
@author: Kir
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_moveit2_flexbe_states.move_group_joint_plan_execute_state import MoveGroupJointPlanExecuteState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class test7SM(Behavior):
    """
    Define test7.

    test
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'test7'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:1318 y:79, x:1318 y:158
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.joint_names = ["shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"]
        _state_machine.userdata.joint_values = [0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0]
        _state_machine.userdata.move_group = 'manipulator'
        _state_machine.userdata.planner_id = ''

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:487 y:222
            OperatableStateMachine.add('MoveGroupJointPlanExecuteState',
                                       MoveGroupJointPlanExecuteState(action_topic='/move_action',
                                                                      move_group=None,
                                                                      joint_names=None,
                                                                      joint_values=None,
                                                                      verbose=False,
                                                                      joint_tolerance=0.0,
                                                                      constraint_weight=1.0,
                                                                      allowed_planning_time=5.0,
                                                                      planner_id='',
                                                                      timeout=10.0),
                                       transitions={'param_error': 'failed'  # 995 211 -1 -1 -1 -1
                                                    , 'planning_failed': 'failed'  # 995 214 -1 -1 -1 -1
                                                    , 'done': 'finished', 'failed': 'failed'  # 995 211 -1 -1 -1 -1
                                                    , 'unavailable': 'failed'  # 995 211 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 995 211 -1 -1 -1 -1
                                                    },
                                       autonomy={'param_error': Autonomy.Off,
                                                 'planning_failed': Autonomy.Off,
                                                 'done': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'unavailable': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'move_group': 'move_group',
                                                  'joint_names': 'joint_names',
                                                  'joint_values': 'joint_values',
                                                  'planner_id': 'planner_id',
                                                  'planned_trajectory': 'planned_trajectory',
                                                  'executed_trajectory': 'executed_trajectory',
                                                  'trajectory_start': 'trajectory_start',
                                                  'planning_time': 'planning_time',
                                                  'error_msg': 'error_msg'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
