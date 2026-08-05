import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.actions import Node

#配置读取
def load_nav_file_config(config_path: str, validate_pcd_file: bool) -> tuple[str, str, str]:  
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {} #把yaml文件转换成python字典

    relocalization_bin_file = os.path.abspath( #把路径转换成绝对路径
        os.path.expanduser(str(config.get("relocalization_bin_file", "")).strip())
    )
    relocalization_pcd_file = os.path.abspath(
        os.path.expanduser(str(config.get("relocalization_pcd_file", "")).strip())
    )
    map_package_dir = os.path.abspath(
        os.path.expanduser(str(config.get("map_package_dir", "")).strip())
    )

    if not relocalization_bin_file:
        raise RuntimeError(f"{config_path} 中未配置 relocalization_bin_file")
    if validate_pcd_file and not relocalization_pcd_file:
        raise RuntimeError(f"{config_path} 中未配置 relocalization_pcd_file")
    if not map_package_dir:
        raise RuntimeError(f"{config_path} 中未配置 map_package_dir")
    if not os.path.isfile(relocalization_bin_file):
        raise RuntimeError(f"重定位 .bin 文件不存在: {relocalization_bin_file}")
    if validate_pcd_file and not os.path.isfile(relocalization_pcd_file):
        raise RuntimeError(f"重定位 .pcd 文件不存在: {relocalization_pcd_file}")
    if not os.path.isdir(map_package_dir):
        raise RuntimeError(f"地图目录不存在: {map_package_dir}")

    return relocalization_bin_file, relocalization_pcd_file, map_package_dir


def load_d1_controller_params(config_path: str) -> dict:  #加载控制器参数
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    return dict(config.get("d1_controller", {}).get("ros__parameters", {}))


def load_bool_config(config_path: str, key: str, default: bool) -> str:
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    return "true" if bool(config.get(key, default)) else "false"


def generate_launch_description():
    # ws = os.environ.get("GO2W_NAV_WS")
    # if ws:
    #     nav_params_config = os.path.join(
    #         ws,
    #         "src",
    #         "jie_3d_nav",
    #         "octo_planner",
    #         "config",
    #         "nav_params.yaml"
    #     )
    # else:
    #     raise RuntimeError("请设置环境变量 GO2W_NAV_WS 指向工作空间路径")
    octo_planner_share = get_package_share_directory("octo_planner")  #获取octo_planner包的目录
    nav_params_config = os.path.join(octo_planner_share, "config", "nav_params.yaml") #配置文件路径
    
    show_rviz_default = load_bool_config(nav_params_config, "show_rviz", False)
    show_map_gui_default = load_bool_config(nav_params_config, "show_map_gui", False)

    # 真正用到的参数在每个节点文件的IfCondition
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",  #rviz  false
        default_value=show_rviz_default,
        description="Launch RViz and RViz-only point cloud publishers",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",  #地图gui false
        default_value=show_map_gui_default,
        description="Launch Qt map viewer and save/load windows",
    )
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",  #路径规划
        default_value="true",
        description="Launch jie_path_node for interactive path planning",
    )
    launch_controller_arg = DeclareLaunchArgument(
        "launch_controller",  #控制器 传入的时候传了true
        default_value="false",
        description="Launch d1_controller for path execution",
    )
    launch_web_arg = DeclareLaunchArgument(
        "launch_web",   #网页端
        default_value="true",
        description="Launch the web-based OctoMap viewer",
    )
    launch_rosbridge_arg = DeclareLaunchArgument(
        "launch_rosbridge",  #ROS2与网页通信桥梁
        default_value="true",
        description="Launch rosbridge_websocket for the web client",
    )
    web_http_port_arg = DeclareLaunchArgument(
        "web_http_port",     #设置网页服务器端口
        default_value="8080",
        description="HTTP port for the web viewer",
    )

    jie_octomap_share = get_package_share_directory("jie_octomap") #获取jie_octomap包的目录

    odin1_loc_rviz_config_file = os.path.join(
        jie_octomap_share, "rviz", "odin1_loc.rviz"    #rviz文件路径
    )
    web_root = os.path.join(jie_octomap_share, "web")  #网页资源目录
    web_server_script = os.path.abspath( #网页服务器脚本路径
        os.path.join(
            jie_octomap_share,
            "..",
            "..",
            "lib",
            "jie_octomap",
            "no_cache_http_server.py",
        )
    )
    relocalization_bin_file, relocalization_pcd_file, map_package_dir = load_nav_file_config(
        nav_params_config,
        validate_pcd_file=(show_rviz_default == "true"),
    )
    d1_controller_params = load_d1_controller_params(nav_params_config)

    robot_desc = """<?xml version="1.0"?>
<robot name="web_test_robot">
  <link name="base_link"/>
</robot>
"""
    go2_share = get_package_share_directory("go2_description")
    common = {"use_sim_time": False}
    robot_state_publisher_node = Node( 
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=
        [
            common,
            {
                # "robot_description": Command(["xacro ", os.path.join(go2_share, "xacro", "robot.xacro")," use_gazebo:=false"])
                "robot_description": robot_desc
            },
        ],
    )

    body_to_base_link_node = Node( 
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
    )

    # Disabled fake odom -> base_link TF.
    # Real chain should be: map -> camera_init -> body -> base_link.
    # static_odom_to_base_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_odom_to_base_link",
    #     output="screen",
    #     arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    # )

    # Disabled fake map -> odom TF.
    # FAST_LIO_LOCALIZATION2 now provides real map -> camera_init.
    # test_map_to_odom_tf_node = Node(
    #     package="octo_planner",
    #     executable="test_map_to_odom_tf_node",
    #     name="test_map_to_odom_tf_node",
    #     output="screen",
    #     parameters=[
    #         {
    #             "parent_frame": "map",
    #             "child_frame": "odom",
    #             "radius": 2.0,
    #             "orbit_period": 20.0,
    #             "spin_rate": 0.8,
    #         }
    #     ],
    # )
    #把pcd文件发布到/pcd_points话题上，rviz订阅这个话题显示点云
    pcd_publisher_node = Node(  
        package="jie_octomap",
        executable="pcd_file_publisher.py",
        name="pcd_file_publisher",
        output="screen",
        parameters=[
            {
                "pcd_path": relocalization_pcd_file,
                "topic": "/pcd_points",
                "frame_id": "map",
                "publish_hz": 1.0,
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    #rviz可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", odin1_loc_rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    #路径规划节点
    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[
            {
                "octomap_topic": "/octomap",  #3D地图话题
                "start_topic": "/start_point",  #起点
                "goal_topic": "/goal_point",    #终点
                "path_topic": "/planned_path",  #规划路径输出话题
                "path_marker_topic": "/planned_path_marker",  #rviz显示路径
                "preblocked_marker_topic": "/preblocked_cells_markers", #rviz显示禁行栅格
                "edited_occupied_marker_topic": "/edited_occupied_markers", #修改后的障碍区域
                "traversable_marker_topic": "/traversable_cells_markers", #可通行区域
                "risk_cost_topic": "/risk_cost_cells", #风险代价图层
                "frame_id": "map",   #地图坐标系
                "map_id": "loaded_map",  #地图包id，用于管理多个地图
                "source_world_file": "", #来源地图文件，空为不从Gazebo world加载
                "robot_radius": 0.25,  #机器人安全半径
                "max_iterations": 500000, #最大搜索次数
                "snap_search_radius_cells": 12, #起点/终点修正范围，若起点/终点在障碍物上，则在该范围内寻找可通行的栅格作为新的起点/终点
                "require_ground_support": True, #要求路径点下面必须有地面
                "strict_direct_ground_support": False, #是否必须正下方有地面
                "ground_support_xy_radius_cells": 1, #检查地面范围
                "ground_support_depth_cells": 1,    #向下检查多少层
                "enable_preblocked_costmap": True,  #是否启动预设危险区域
                "preblocked_costmap_radius_cells": 3, #预设危险区域半径
                "preblocked_costmap_weight": 2.5,     #预设危险区域权重
            }
        ],
    )

    #接收path规划节点发布的路径，控制机器人运动
    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        parameters=[
            d1_controller_params,
            {
                "path_topic": "/planned_path",
                "start_navigation_topic": "/start_navigation",
                "stop_navigation_topic": "/stop_navigation",
                "require_start_command": True,
                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/web_cmd_vel",
                "tracking_point_marker_topic": "/tracking_point_marker",
                "map_frame": "map",
                "base_frame": "base_link",
            }
        ],
    )
    #地图资源管理节点
    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {
                "autoload_package_path": map_package_dir,
            }
        ],
    )
    #把 /octomap 转换成 /octomap_occupied_markers，即把八叉树地图转换成rviz可视化的marker
    occupied_marker_node = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )
    #桌面地图查看器GUI,支持：打开地图包、刷新地图、保存地图、查看占据、禁行、可通行、风险代价图层、编辑栅格、选择起点、终点、导航目标
    map_viewer_gui_node = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
        additional_env={"MAP_VIEWER_DEFAULT_PACKAGE": map_package_dir},
        parameters=[
            {
                "tf_parent_frame": "map",
                "tf_child_frame": "base_link",
            }
        ],
    )
    #地图保存与加载GUI
    map_save_gui_node = Node(
        package="jie_octomap",
        executable="map_save_gui",
        name="map_save_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )
    #网页服务器，提供octomap的web可视化
    web_http_server = ExecuteProcess(
        cmd=[
            web_server_script,
            "--port",
            LaunchConfiguration("web_http_port"),
            "--directory",
            web_root,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_web")),
    )
    # ROS2与网页通信桥梁，提供websocket服务
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
    )

    return LaunchDescription(
        [
            launch_rviz_arg,
            launch_map_gui_arg,
            launch_planner_arg,
            launch_controller_arg,
            launch_web_arg,
            launch_rosbridge_arg,
            web_http_port_arg,
            robot_state_publisher_node,
            pcd_publisher_node,
            rviz_node,
            planner_node,
            controller_node,
            map_package_manager_node,
            occupied_marker_node,
            map_viewer_gui_node,
            map_save_gui_node,
            web_http_server,
            rosbridge_node,
        ]
    )
