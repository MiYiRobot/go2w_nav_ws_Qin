from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #点云地图文件地址和话题名称参数
    pcd_map_path = LaunchConfiguration("map")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")

    declare_map_path = DeclareLaunchArgument(
        "map",
        default_value="/home/luo/go2w_nav_ws_Qin/maps/localization/start_aligned_to_big.pcd",
        description="Path to PCD map file"
    )

    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic",
        default_value="/map",
        description="Topic to publish PCD map"
    )

    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{
            "map_voxel_size": 0.4,   #体素滤波器的体素大小，单位为米，0.4m每个体素
            "scan_voxel_size": 0.1,  #采样粒度
            "freq_localization": 0.5,#定位更新频率
            "freq_global_map": 0.25, #全局地图更新频率（没用到，作为保留参数）
            "localization_threshold": 0.8, #ICP fitness 的门限。匹配结果的 fitness 高于这个值，才认为这次全局定位成功
            "fov": 6.28319,  #2π，扫描范围
            "fov_far": 200,  #地图搜索范围，单位为米
            "pcd_map_path": pcd_map_path,#路径地址
            "pcd_map_topic": pcd_map_topic#话题
        }],
    )

    transform_fusion_node = Node(  #tf融合节点，将全局定位结果和里程计结果融合，得到更平滑的位姿
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
    )

    pcd_publisher_node = Node(   #把.pcd地图文件发布为点云话题
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{
            "file_name": pcd_map_path,
            "tf_frame": "map",   #点云话题的坐标系
            "cloud_topic": pcd_map_topic,
            "period_ms_": 500    #发布周期：500ms
        }],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    return LaunchDescription([
        declare_map_path,
        declare_pcd_map_topic,
        pcd_publisher_node,
        global_localization_node,
        transform_fusion_node,
    ])