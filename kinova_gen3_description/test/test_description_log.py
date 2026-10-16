"""description.launch.py logs the embodiment line BEFORE it starts anything, so the
active robot is the first thing in the log. (LogInfo goes to launch's own logger, which
launch_testing's proc_output cannot see -- hence this offline check.)"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import LogInfo
from launch_ros.actions import Node

TILTED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mounts", "tilted.yaml")


def _launch_module():
    path = os.path.join(
        get_package_share_directory("kinova_gen3_description"),
        "launch",
        "description.launch.py",
    )
    spec = importlib.util.spec_from_file_location("description_launch", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_embodiment_line_comes_first():
    ctx = LaunchContext()
    ctx.launch_configurations["mount_file"] = TILTED
    actions = _launch_module()._rsp(ctx)
    assert isinstance(actions[0], LogInfo)
    assert "".join(s.perform(ctx) for s in actions[0].msg) == (
        "embodiment: test_tilted (mount rpy_deg [0.0, 30.0, 0.0])"
    )
    assert isinstance(actions[1], Node)
