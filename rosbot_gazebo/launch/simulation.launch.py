# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modifications Copyright (c) 2026 [Wut Yee Oo]

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    config_dir = LaunchConfiguration("config_dir")
    rviz = LaunchConfiguration("rviz")
    gz_gui = LaunchConfiguration("gz_gui")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    
    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rosbot_gazebo"), "config", "teleop_with_estop.config"]
        ),
        description="Run simulation with specific GUI layout.",
    )

    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    namespaced_gz_gui = ReplaceString(
        source_file=gz_gui,
        replacements={"{namespace}": namespace},
    )

    gz_headless_rendering = LaunchConfiguration("gz_headless_rendering")
    declare_gz_headless_rendering_arg = DeclareLaunchArgument(
        "gz_headless_rendering",
        default_value=EnvironmentVariable("GZ_HEADLESS_RENDERING", default_value="False"),
        description=(
            "Run Gazebo server-only with headless rendering and OGRE v1. "
            "Workaround for aarch64/Parallels VirGL where ogre2 GLSL shaders "
            "fail to compile. Leave False on amd64 + GPU."
        ),
        choices=["True", "true", "False", "false"],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_gui": namespaced_gz_gui,"gz_log_level": "1"}.items(),
        # condition=UnlessCondition(gz_headless_rendering),
    )

    # Headless-rendering path: call ros_gz_sim directly, bypassing the wrapper
    # that hardcodes ogre2. OGRE v1 is used because ogre2 shaders fail under VirGL.
    # gz_world = PathJoinSubstitution(
    #     [FindPackageShare("husarion_gz_worlds"), "worlds", "husarion_world.sdf"]
    # )
    # gz_sim_headless = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    #     ),
    #     launch_arguments={
    #         "gz_args": ["--headless-rendering -s --render-engine-server ogre -r -v 1 ", gz_world],
    #         "on_exit_shutdown": "true",
    #     }.items(),
    #     condition=IfCondition(gz_headless_rendering),
    # )

    config_rosbot_gazebo_dir = PythonExpression(
        [
            "'",
            config_dir,
            "/rosbot_gazebo' if '",
            config_dir,
            "' else '",
            FindPackageShare("rosbot_gazebo"),
            "'",
        ]
    )

    gz_bridge_config = PathJoinSubstitution([config_rosbot_gazebo_dir, "config", "gz_bridge.yaml"])
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_gazebo"),
                    "launch",
                    "spawn_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": robot_model,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        launch_arguments={"namespace": ""}.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            declare_config_dir_arg,
            declare_rviz_arg,
            declare_gz_gui,
            declare_namespace_arg,
            declare_robot_model_arg,
            SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            # gz_sim_headless,
            gz_bridge,
            spawn_robot,
            rviz_launch,
        ]
    )
