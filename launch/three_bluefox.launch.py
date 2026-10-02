
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
def generate_launch_description():
    declare_uav_name = DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='UAV namespace'
    )
    single_camera_launch = PathJoinSubstitution([
        FindPackageShare('uvdar_core'),
        'launch',
        'single_bluefox.launch.py'
    ])
    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_camera_launch),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'camera_name': 'left',
            'device': EnvironmentVariable('BLUEFOX_LEFT_ID'),
            'expose_us': EnvironmentVariable('EXPOSE_US_LEFT'),
            'aec': 'false',  # manual exposure - see note below
            'standalone': 'true',
        }.items()
    )
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_camera_launch),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'camera_name': 'right',
            'device': EnvironmentVariable('BLUEFOX_RIGHT_ID'),
            'expose_us': EnvironmentVariable('EXPOSE_US_RIGHT'),
            'aec': 'false',
            'standalone': 'true',
        }.items()
    )
    back_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_camera_launch),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'camera_name': 'back',
            'device': EnvironmentVariable('BLUEFOX_BACK_ID'),
            'expose_us': EnvironmentVariable('EXPOSE_US_BACK'),
            'aec': 'false',
            'standalone': 'true',
        }.items()
    )
    uvcam_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=['uvcam_left_tf_', LaunchConfiguration('uav_name')],
        arguments=[
            '--x', '0.03',
            '--y', '0.10',
            '--z', '0.06',
            '--yaw', '-0.3490658504',
            '--pitch', '0.0',
            '--roll', '-1.57079632679',
            '--frame-id', [LaunchConfiguration('uav_name'), '/fcu'],
            '--child-frame-id', [LaunchConfiguration('uav_name'), '/bluefox_left'],
        ]
    )
    uvcam_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=['uvcam_right_tf_', LaunchConfiguration('uav_name')],
        arguments=[
            '--x', '0.03',
            '--y', '-0.10',
            '--z', '0.06',
            '--yaw', '-2.792526803',
            '--pitch', '0.0',
            '--roll', '-1.57079632679',
            '--frame-id', [LaunchConfiguration('uav_name'), '/fcu'],
            '--child-frame-id', [LaunchConfiguration('uav_name'), '/bluefox_right'],
        ]
    )
    # Aft-looking camera. The child frame here is the string
    # single_bluefox.launch.py builds for camera_name 'back' ($UAV_NAME +
    # '/bluefox_back', lines 134-137) and the string three_bluefox_bearing.yaml states
    # as the `back` input's camera_frame. Those three have to be one string: a bearing
    # stamped in a frame no mount publishes is not an error, it is a target RViz draws
    # at the origin with nothing on stderr.
    uvcam_back_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=['uvcam_back_tf_', LaunchConfiguration('uav_name')],
        arguments=[
            '--x', '-0.10',
            '--y', '0.0',
            '--z', '0.06',
            '--yaw', '0.0',
            '--pitch', '1.57079632679',
            '--roll', '3.14159265359',
            '--frame-id', [LaunchConfiguration('uav_name'), '/fcu'],
            '--child-frame-id', [LaunchConfiguration('uav_name'), '/bluefox_back'],
        ]
    )
    # The stages that turn those three image streams into bearings: detector, tracker
    # and bearing, one process each, all three iterating over the three inputs of
    # config/three_bluefox_bearing.yaml.
    #
    # two_bluefox.launch.py does not do this - it starts cameras only, and whoever runs
    # it starts bearing.launch.py themselves. This file launches both so that it is the
    # one command a vehicle needs: whatever consumes the bearings downstream can then
    # assume somebody is publishing them.
    #
    # The config is passed explicitly because bearing.launch.py's own default is
    # default_bearing.yaml, which describes two cameras. Started with that, this rig
    # silently has no `back` camera: two images read, two bearings published, and
    # nothing on stderr about the third being ignored.
    bearing_endpoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uvdar_core'), 'launch', 'bearing.launch.py',
        ])),
        launch_arguments={
            'namespace': LaunchConfiguration('uav_name'),
            'config_file': PathJoinSubstitution([
                FindPackageShare('uvdar_core'), 'config', 'three_bluefox_bearing.yaml',
            ]),
        }.items()
    )
    return LaunchDescription([
        declare_uav_name,
        left_camera,
        right_camera,
        back_camera,
        uvcam_left_tf,
        uvcam_right_tf,
        uvcam_back_tf,
        bearing_endpoint,
    ])
