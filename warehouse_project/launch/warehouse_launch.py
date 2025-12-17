import os
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)

def generate_launch_description(): 
    # Definisco percorso al file del mondo e ai modelli
    world_file = os.path.join(get_package_share_directory("warehouse_project"), "worlds", "warehouse.world")  
    models_path = os.path.join(get_package_share_directory('warehouse_project'), 'models')

    declared_arguments = [
        DeclareLaunchArgument("gz_args",
            default_value=["-r ", world_file],
            description="path to world file",
        )
    ]
    
    # Start Gazebo
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch",                    
                                      'gz_sim.launch.py'])]),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args"),
                              "publish_clock": "true",
                              }.items()
    )

    # Clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    #--------------------------------------
    #--------------IIWA ROBOT--------------
    #--------------------------------------
    iiwa_xacro_file_name = "iiwa.config.xacro"
    iiwa_xacro = os.path.join(
        get_package_share_directory("iiwa_description"), "config", iiwa_xacro_file_name)

    iiwa_params={"robot_description": Command(["xacro ", iiwa_xacro, " namespace:=iiwa/"])}

    iiwa_rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="iiwa",
        output="screen",
        parameters=[iiwa_params, {"use_sim_time": True}],
    )

    iiwa_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/iiwa/robot_description", "-name", "iiwa"],
    )

    iiwa_jsb_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa",
        arguments=["joint_state_broadcaster", "--controller-manager", "/iiwa/controller_manager"],
    )

    iiwa_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="iiwa",
        arguments=["velocity_controller", "--controller-manager", "/iiwa/controller_manager"],
    )

    spawn_jsb_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_spawn, 
            on_exit=[iiwa_jsb_node],
        )
    )

    spawn_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=iiwa_jsb_node,
            on_exit=[iiwa_robot_controller_spawner],
        )
    )

    # Camera bridge 
    iiwa_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="iiwa",
        arguments=[
            "iiwa/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "iiwa/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "--ros-args",
            "-r", "iiwa/camera:=/stereo/left/image_rect_color",
            "-r", "iiwa/camera_info:=/stereo/left/camera_info",
        ],
        output="screen",
    )
    
    rqt_image = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        arguments=["/stereo/left/image_rect_color"],
    )
    

    # STATIC TRANSFORM PER FRAME OTTICO (IIWA)
    optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_link_optical'],
        output='screen'
    )
    
    bridge_service = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/warehouse/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        output='screen'
    )
    

    # =========================================================================
    #  GRIPPER BRIDGE (Detachable Joint)
    # =========================================================================
    gripper_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gripper_bridge',
        arguments=[
            # MEDICINE
            '/model/iiwa/detachable_joint/box_medicine/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/model/iiwa/detachable_joint/box_medicine/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
            
            # TOYS
            '/model/iiwa/detachable_joint/box_toys/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/model/iiwa/detachable_joint/box_toys/detach@std_msgs/msg/Empty@ignition.msgs.Empty',

            # CLOTHES
            '/model/iiwa/detachable_joint/box_clothes/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/model/iiwa/detachable_joint/box_clothes/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # NODI ARUCO ROS - PER IIWA 
    # -------------------------------------------------------------------------
    aruco_remaps_iiwa = [
        ('/camera_info', '/stereo/left/camera_info'),
        ('/image', '/stereo/left/image_rect_color')
    ]
    
    aruco_node_med = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_med',
        namespace='aruco_medicine',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,
            'marker_id': 1,
            'reference_frame': 'world',
            'camera_frame': 'camera_link_optical',  
            'marker_frame': 'marker_med_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_iiwa,
        output='screen'
    )

    aruco_node_clothes = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_clothes',
        namespace='aruco_clothes',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,
            'marker_id': 3,
            'reference_frame': 'world',
            'camera_frame': 'camera_link_optical', 
            'marker_frame': 'marker_clothes_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_iiwa,
        output='screen'
    )

    aruco_node_toys = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_toys',
        namespace='aruco_toys',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,
            'marker_id': 2,
            'reference_frame': 'world',
            'camera_frame': 'camera_link_optical', 
            'marker_frame': 'marker_toys_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_iiwa,
        output='screen'
    )

    #--------------------------------------
    #------------FRA2MO ROBOT--------------
    #--------------------------------------
    fra2mo_xacro_file_name = "fra2mo.urdf.xacro"
    fra2mo_xacro = os.path.join(get_package_share_directory('ros2_fra2mo'), "urdf", fra2mo_xacro_file_name)
    fra2mo_models_path = os.path.join(get_package_share_directory('ros2_fra2mo'), 'models')
    
    fra2mo_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', fra2mo_xacro]),value_type=str)}
    
    fra2mo_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='fra2mo',
        parameters=[fra2mo_description_xacro, {"use_sim_time": True}]
    )
    
    fra2mo_jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )
 
    position = [0.15, -19.31, 0.0] 

    fra2mo_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/fra2mo/robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),
                    # "-Y", str(yaw)
                   ]
    )
 
    fra2mo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='fra2mo',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   #'/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/fra2mo/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/fra2mo/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        remappings=[('/cmd_vel', 'cmd_vel'), 
                    #('/model/fra2mo/odometry', 'odom'), 
                    #('/model/fra2mo/tf', 'tf'), 
                    ('/lidar', 'lidar'),
                    ('/fra2mo/camera', 'camera'),
                    ('/fra2mo/camera_info', 'camera_info')],
        output='screen'
    )
 
    fra2mo_odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )
    
    
    fra2mo_gripper_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fra2mo_gripper_bridge',
        arguments=[
            # MEDICINE
            '/model/fra2mo/detachable_joint/box_medicine/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/model/fra2mo/detachable_joint/box_medicine/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
            # TOYS
            '/model/fra2mo/detachable_joint/box_toys/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/model/fra2mo/detachable_joint/box_toys/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
            # CLOTHES
            '/model/fra2mo/detachable_joint/box_clothes/attach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/model/fra2mo/detachable_joint/box_clothes/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
        ],
        output='screen'
    )
    
    # FIX TF OTTICO PER FRA2MO
    fra2mo_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '-1.5708', '0', '-1.5708', 
            'fra2mo_camera_link',         
            'fra2mo_camera_link_optical'   
        ],
        output='screen'
    )
    
    # -------------------------------------------------------------------------
    # CONFIGURAZIONE ARUCO PER FRA2MO 
    # -------------------------------------------------------------------------
    aruco_remaps_fra2mo = [
        ('/camera_info', '/fra2mo/camera_info'),
        ('/image', '/fra2mo/camera')
    ]

    # NODO ARUCO START (Vicino a IIWA) - ID 11
    aruco_node_fra2mo_start = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_iiwa',
        namespace='aruco_iiwa',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,
            'marker_id': 11,
            'reference_frame': '',
            'camera_frame': 'fra2mo_camera_link_optical', 
            'marker_frame': 'marker_iiwa_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_fra2mo,
        output='screen'
    )

    # NODO ARUCO ROSSO (Zona 1) - ID 12
    aruco_node_red = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_red',
        namespace='aruco_red',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,
            'marker_id': 12,
            'reference_frame': '',
            'camera_frame': 'fra2mo_camera_link_optical',
            'marker_frame': 'marker_red_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_fra2mo,
        output='screen'
    )

    # NODO ARUCO VERDE (Zona 2) - ID 13
    aruco_node_green = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_green',
        namespace='aruco_green',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,
            'marker_id': 13,
            'reference_frame': '',
            'camera_frame': 'fra2mo_camera_link_optical', 
            'marker_frame': 'marker_green_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_fra2mo,
        output='screen'
    )

    # NODO ARUCO BLU (Zona 3) - ID 14
    aruco_node_blue = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single_blue',
        namespace='aruco_blue',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,
            'marker_id': 14,
            'reference_frame': '',
            'camera_frame': 'fra2mo_camera_link_optical', 
            'marker_frame': 'marker_blue_frame',
            'corner_refinement': 'LINES',
            'use_sim_time': True
        }],
        remappings=aruco_remaps_fra2mo,
        output='screen'
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = models_path + ':' + fra2mo_models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
    ] + declared_arguments + [ 
        gazebo_ignition,
        clock_bridge,
        
        # IIWA Base
        iiwa_rsp_node,
        iiwa_spawn,
        spawn_jsb_handler,
        spawn_controller_handler,
        iiwa_bridge_camera,
        optical_tf_node,      
        gripper_bridge, 
        rqt_image,
        bridge_service,
        
        # Fra2Mo Base
        fra2mo_rsp_node,
        fra2mo_jsp_node,
        fra2mo_spawn,
        fra2mo_bridge,
        fra2mo_odom_tf,
        fra2mo_gripper_bridge,
        fra2mo_optical_tf_node, 
        
        # Nodi IIWA
	aruco_node_med,
	aruco_node_clothes,
	aruco_node_toys,   
                   
        # Nodi Fra2Mo
        aruco_node_fra2mo_start,
        aruco_node_red,
        aruco_node_green,
        aruco_node_blue,
    ])
