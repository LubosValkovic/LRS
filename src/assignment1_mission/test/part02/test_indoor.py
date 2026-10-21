# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Mission resume regression when the first CSV row differs from launch."""

from unittest.mock import Mock

from assignment1_mission.part02.indoor import IndoorMission


def test_landtakeoff_does_not_repeat_launch_to_first_waypoint(flight_factory, clock):
    node = flight_factory(IndoorMission)
    node.segment = 3
    node.pre_route = [(13., 7., 1.), (12., 6., 1.)]
    node._begin_segment = Mock()
    node._aim_pre = Mock()
    node.after_takeoff()
    node._begin_segment.assert_called_once_with(turn=True)
    node._aim_pre.assert_not_called()
