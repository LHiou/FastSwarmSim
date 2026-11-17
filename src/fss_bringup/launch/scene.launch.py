"""根据场景 YAML 启动共享时钟、各架无人机、LiDAR 和机体可视化。"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_scene(context):
    """读取启动时配置，为每架飞机分配独立话题与 TF 名称。"""
    config_path = Path(LaunchConfiguration("scene").perform(context))
    with config_path.open(encoding="utf-8") as stream:
        scene = yaml.safe_load(stream)
    px4_share = Path(get_package_share_directory("fss_px4_sim"))
    sensing_share = Path(get_package_share_directory("fss_sensing"))
    use_time = scene.get("use_sim_time", True)
    mode = scene.get("mode", "px4")
    if mode not in ("px4", "ideal"):
        print("场景 mode 必须为 px4 或 ideal")
        return []
    names = [vehicle["name"] for vehicle in scene["vehicles"]]
    if not names or len(set(names)) != len(names):
        print("场景必须包含车辆，且车辆名称不能重复")
        return []
    actions = []
    if use_time:
        actions.append(Node(
            package="fss_time", executable="time_coordinator", name="fss_time_coordinator",
            parameters=[{"use_sim_time": True, "publish_clock": True,
                         "max_real_time_factor": float(scene.get("real_time_factor", 1.0)),
                         "auto_start": True}], output="screen"))
    lidar_config = str(sensing_share / "config/local_pointcloud_sim.yaml")
    for index, vehicle in enumerate(scene["vehicles"], start=1):
        name = vehicle["name"]
        x, y, z = map(float, vehicle.get("position", [0, 0, 0]))
        yaw = float(vehicle.get("yaw_deg", 0.0))
        common = {"use_sim_time": use_time, "use_fss_sim_time": use_time}
        if mode == "px4":
            executable = "px4_rotor_sim_node"
            parameters = [str(px4_share / "config/px4_params.yaml"), common, {
                "init_x_East_metre": x, "init_y_North_metre": y, "init_z_Up_metre": z,
                "init_yaw_deg": yaw, "MAV_SYS_ID": index,
                "px4.mavlink_transport": "direct_ros",
                "local_position.child_frame_id": f"{name}/base_link"}]
        else:
            executable = "perfect_mavros_quadrotor_sim_node"
            # 原生 init_yaw 参数接收角度，内部再换算弧度。
            parameters = [common, {"init_x": x, "init_y": y, "init_z": z, "init_yaw": yaw,
                                   "pose_publish_rate": 60.0, "odom_publish_rate": 60.0}]
        actions.append(Node(package="fss_px4_sim", executable=executable, namespace=name,
                            parameters=parameters, output="screen"))
        if scene.get("enable_sensing", True):
            actions.append(Node(
                package="fss_sensing", executable="local_pointcloud_sim_node", namespace=name,
                parameters=[common, {"config_path": lidar_config}], output="screen"))
        if scene.get("enable_visualizer", True):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(px4_share / "launch/drone_visualizer_single.launch.py")),
                launch_arguments={"namespace": name, "use_sim_time": str(use_time).lower(),
                                  "visualize_max_freq": "60.0", "visualize_path_time": "30.0",
                                  "vis.color": vehicle.get("color", "red")}.items()))
    return actions


def generate_launch_description():
    """声明 scene 参数，默认使用包内的单机 PX4 场景。"""
    default_scene = Path(get_package_share_directory("fss_bringup")) / "config/scene.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value=str(default_scene)),
        OpaqueFunction(function=create_scene),
    ])
