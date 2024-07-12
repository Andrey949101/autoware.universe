# Copyright 2021 The Autoware Foundation.
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

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def launch_setup(context, *args, **kwargs):
    # vehicle information param path
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    vehicle_characteristics_param_path = LaunchConfiguration(
        "vehicle_characteristics_param_file"
    ).perform(context)
    with open(vehicle_characteristics_param_path, "r") as f:
        vehicle_characteristics_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    simulator_model_param_path = LaunchConfiguration("simulator_model_param_file").perform(context)
    simulator_model_param = launch_ros.parameter_descriptions.ParameterFile(
        param_file=simulator_model_param_path, allow_substs=True
    )

    simple_planning_simulator_node = Node(
        package="simple_planning_simulator",
        executable="simple_planning_simulator_exe",
        name="simple_planning_simulator",
        namespace="simulation",
        output="screen",
        parameters=[
            vehicle_info_param,
            vehicle_characteristics_param,
            simulator_model_param,
            {
                "initial_engage_state": LaunchConfiguration("initial_engage_state"),
            },
        ],
        remappings=[
            ("input/vector_map", "/map/vector_map"),
            ("input/initialpose", "/initialpose3d"),
            ("input/ackermann_control_command", "/control/command/control_cmd"),
            ("input/actuation_command", "/control/command/control_cmd"),
            ("input/manual_ackermann_control_command", "/vehicle/command/manual_control_cmd"),
            ("input/gear_command", "/control/command/gear_cmd"),
            ("input/manual_gear_command", "/vehicle/command/manual_gear_command"),
            ("input/turn_indicators_command", "/control/command/turn_indicators_cmd"),
            ("input/hazard_lights_command", "/control/command/hazard_lights_cmd"),
            ("input/trajectory", "/planning/scenario_planning/trajectory"),
            ("input/engage", "/vehicle/engage"),
            ("input/control_mode_request", "/control/control_mode_request"),
            ("output/twist", "/vehicle/status/velocity_status"),
            ("output/odometry", "/localization/kinematic_state"),
            ("output/acceleration", "/localization/acceleration"),
            ("output/imu", "/sensing/imu/imu_data"),
            ("output/steering", "/vehicle/status/steering_status"),
            ("output/gear_report", "/vehicle/status/gear_status"),
            ("output/turn_indicators_report", "/vehicle/status/turn_indicators_status"),
            ("output/hazard_lights_report", "/vehicle/status/hazard_lights_status"),
            ("output/control_mode_report", "/vehicle/status/control_mode"),
        ],
    )
    launch_vehicle_cmd_converter = True # tmp

    # Determine if we should launch raw_vehicle_cmd_converter based on the vehicle_model_type
    with open(simulator_model_param_path, "r") as f:
        simulator_model_param_yaml = yaml.safe_load(f)
    launch_vehicle_cmd_converter = (
        simulator_model_param_yaml["/**"]["ros__parameters"].get("vehicle_model_type") == "ACTUATION_CMD"
    )
    # 1) Launch only simple_planning_simulator_node
    if not launch_vehicle_cmd_converter:
        return [simple_planning_simulator_node]
    # 2) Launch raw_vehicle_cmd_converter too
    vehicle_launch_pkg = LaunchConfiguration("vehicle_model").perform(context) + "_launch"
    raw_vehicle_converter_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [FindPackageShare("autoware_raw_vehicle_cmd_converter"), "/launch/raw_vehicle_converter.launch.xml"]
        ),
        launch_arguments={
            'config_file': [FindPackageShare(vehicle_launch_pkg), '/config/raw_vehicle_cmd_converter.param.yaml']
        }.items(),
    )
    return [simple_planning_simulator_node, raw_vehicle_converter_node]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("autoware_vehicle_info_utils"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    add_launch_arg(
        "vehicle_characteristics_param_file",
        [
            FindPackageShare("simple_planning_simulator"),
            "/param/vehicle_characteristics.param.yaml",
        ],
        "path to config file for vehicle characteristics",
    )

    add_launch_arg(
        "simulator_model_param_file",
        [
            FindPackageShare("simple_planning_simulator"),
            "/param/simple_planning_simulator_default.param.yaml",
        ],
        "path to config file for simulator_model",
    )

    add_launch_arg(
        "acceleration_param_file",
        [
            FindPackageShare("simple_planning_simulator"),
            "/param/acceleration_map.csv",
        ],
    )

    # NOTE: {vehicle_model}_launchにはcsv_accel_brake_map_pathを渡す必要がある。

    add_launch_arg(
        "csv_accel_brake_map_path",
        [
            FindPackageShare("autoware_raw_vehicle_cmd_converter"),
            "/data/default",
        ],
    )

#   <group if="$(var launch_dummy_vehicle)">
#     <arg name="simulator_model" default="$(var vehicle_model_pkg)/config/simulator_model.param.yaml" description="path to the file of simulator model"/>
#     <include file="$(find-pkg-share simple_planning_simulator)/launch/simple_planning_simulator.launch.py">
#       <arg name="vehicle_info_param_file" value="$(var vehicle_info_param_file)"/>
#       <arg name="simulator_model_param_file" value="$(var simulator_model)"/>
#       <arg name="initial_engage_state" value="$(var initial_engage_state)"/>
#     </include>
#   </group>    

    # vehicle_model_pkgの引数をうけとる


    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
