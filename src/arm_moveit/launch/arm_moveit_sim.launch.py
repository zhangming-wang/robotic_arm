import os
import sys

sys.path.append(os.path.dirname(__file__))

from moveit_launch_common import build_moveit_launch_description


def generate_launch_description():
    return build_moveit_launch_description(
        bringup_launch_file="arm_description_sim.launch.py",
        xacro_args=[" use_sim:=true"],
        use_sim_time=True,
    )
