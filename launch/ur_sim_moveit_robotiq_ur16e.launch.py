"""
ur_sim_moveit_robotiq_ur16e.launch.py

UR16e + Robotiq 2F-85 + 테스트베드 시뮬레이션 런치 파일.

기존 ur_sim_moveit_robotiq.launch.py (UR5e) 와의 차이점:
  - description_file: ur16e_robotiq_2f85.urdf.xacro 사용
  - controllers_file: ur16e_robotiq_2f85_controllers.yaml 사용
  - UR 설정 파일 경로: ur_description/config/ur16e/ (중복 없이 직접 참조)
  - world_file 기본값: ur_setup_bringup/worlds/testbed.sdf
  - ur_type 기본값: ur16e
  - 테스트베드 치수는 xacro 파일 기본값을 그대로 사용 (런치 인수 없음)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    # ------------------------------------------------------------------ #
    # UR + 시뮬레이션 기본 인수
    # ------------------------------------------------------------------ #
    ur_type           = LaunchConfiguration("ur_type")
    safety_limits     = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix            = LaunchConfiguration("prefix")

    # MoveIt / 시간 인수
    use_sim_time                     = LaunchConfiguration("use_sim_time")
    launch_rviz                      = LaunchConfiguration("launch_rviz")
    launch_servo                     = LaunchConfiguration("launch_servo")
    moveit_joint_limits_file         = LaunchConfiguration("moveit_joint_limits_file")
    warehouse_sqlite_path            = LaunchConfiguration("warehouse_sqlite_path")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

    # Gazebo 인수
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    # MoveIt 설정 패키지/파일
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    srdf_package          = LaunchConfiguration("srdf_package")
    srdf_file             = LaunchConfiguration("srdf_file")

    # ------------------------------------------------------------------ #
    # 경로 설정
    # ------------------------------------------------------------------ #
    description_package = "ur_setup_bringup"
    description_file    = "ur16e_robotiq_2f85.urdf.xacro"
    runtime_config_package = "ur_setup_bringup"
    controllers_file    = "ur16e_robotiq_2f85_controllers.yaml"

    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # UR16e 설정 파일: ur_description 패키지의 ur16e 폴더를 직접 참조
    # (ur_setup_bringup/config/ur16e/ 로 복사하지 않아 중복 방지)
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur16e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur16e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur16e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur16e", "visual_parameters.yaml"]
    )
    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "initial_positions.yaml"]
    )

    # ------------------------------------------------------------------ #
    # robot_description 빌드 (xacro → URDF 문자열)
    # ------------------------------------------------------------------ #
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ur_type:=",               ur_type,
            " joint_limit_params:=",    joint_limit_params,
            " kinematics_params:=",     kinematics_params,
            " physical_params:=",       physical_params,
            " visual_params:=",         visual_params,
            " initial_positions_file:=", initial_positions_file,
            " safety_limits:=",         safety_limits,
            " safety_pos_margin:=",     safety_pos_margin,
            " safety_k_position:=",     safety_k_position,
            " name:=ur",
            " prefix:=",               prefix,
            " sim_ignition:=true",
            " simulation_controllers:=", controllers_file_path,
            # 테스트베드 치수는 xacro 파일(ur16e_robotiq_2f85.urdf.xacro,
            # testbed.urdf.xacro)의 xacro:arg / params 기본값을 그대로 사용.
            # 변경이 필요하면 해당 xacro 파일의 default 값을 수정할 것.
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ------------------------------------------------------------------ #
    # robot_state_publisher
    # ------------------------------------------------------------------ #
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ------------------------------------------------------------------ #
    # Gazebo 리소스 경로 설정 (Robotiq 메시 URI 해석용)
    # ------------------------------------------------------------------ #
    robotiq_share_parent = PathJoinSubstitution(
        [FindPackageShare("robotiq_description"), ".."]
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            robotiq_share_parent,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            robotiq_share_parent,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    # ------------------------------------------------------------------ #
    # Gazebo 실행 (GUI 유무 분기)
    # ------------------------------------------------------------------ #
    gz_launch_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # ------------------------------------------------------------------ #
    # Gazebo 에 UR16e + 테스트베드 엔티티 스폰
    # URDF 전체(로봇 + 페데스탈 + 플레이트)를 하나의 엔티티로 스폰함.
    # ------------------------------------------------------------------ #
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "ur16e_testbed",
            "-allow_renaming", "true",
        ],
    )

    # ------------------------------------------------------------------ #
    # /clock 브리지 (Ignition → ROS 2)
    # ------------------------------------------------------------------ #
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 컨트롤러 스포너
    # ------------------------------------------------------------------ #
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller       = LaunchConfiguration("initial_joint_controller")
    start_joint_controller         = LaunchConfiguration("start_joint_controller")

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # 시뮬레이션에서는 robotiq_activation_controller 없이 gripper controller 만 스폰
    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    )

    # ------------------------------------------------------------------ #
    # MoveIt 설정
    # ------------------------------------------------------------------ #
    # SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(srdf_package), "srdf", srdf_file]
            ),
            " name:=ur",
            " prefix:=", prefix,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    publish_robot_description_semantic_param = {
        "publish_robot_description_semantic": publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_setup_bringup"), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # OMPL 플래닝 파이프라인
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 트래젝터리 실행 설정
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    if context.perform_substitution(use_sim_time) == "true":
        # 시뮬레이션에서는 joint_trajectory_controller 를 기본으로 사용
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers":                          False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin":  0.5,
        "trajectory_execution.allowed_start_tolerance":       0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene":    True,
        "publish_geometry_updates":  True,
        "publish_state_updates":     True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host":   warehouse_sqlite_path,
    }

    # ------------------------------------------------------------------ #
    # move_group 노드
    # ------------------------------------------------------------------ #
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic_param,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # ------------------------------------------------------------------ #
    # RViz (MoveIt 뷰어)
    # ------------------------------------------------------------------ #
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_setup_bringup"), "rviz", "robot_model.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------ #
    # MoveIt Servo 노드
    # ------------------------------------------------------------------ #
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        condition=IfCondition(launch_servo),
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Gazebo + 컨트롤러 안정화 후 MoveIt / RViz / Servo 지연 실행
    # ------------------------------------------------------------------ #
    moveit_start = TimerAction(
        period=5.0,
        actions=[move_group_node, rviz_node, servo_node],
    )

    return [
        set_ign_resource_path,
        set_gz_resource_path,
        gz_launch_with_gui,
        gz_launch_without_gui,
        gz_sim_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        initial_joint_controller_spawner_stopped,
        robotiq_gripper_controller_spawner,
        moveit_start,
    ]


def generate_launch_description():
    declared_arguments = []

    # ------------------------------------------------------------------ #
    # UR 로봇 인수
    # ------------------------------------------------------------------ #
    declared_arguments.append(DeclareLaunchArgument(
        "ur_type", default_value="ur16e",
        description="UR 로봇 모델 (ur16e 권장).",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_limits", default_value="true",
        description="안전 한계 컨트롤러 활성화 여부.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_pos_margin", default_value="0.15",
        description="안전 컨트롤러의 하한/상한 여유 범위.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_k_position", default_value="20",
        description="안전 컨트롤러의 k-position 인수.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "prefix", default_value='""',
        description="조인트 이름 접두사.",
    ))

    # 컨트롤러 인수
    declared_arguments.append(DeclareLaunchArgument(
        "start_joint_controller", default_value="true",
        description="시작 시 joint controller 를 자동으로 활성화할지 여부.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "initial_joint_controller", default_value="joint_trajectory_controller",
        description="기동할 초기 joint controller 이름.",
    ))

    # Gazebo 인수
    declared_arguments.append(DeclareLaunchArgument(
        "gazebo_gui", default_value="true",
        description="Gazebo GUI 표시 여부.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_setup_bringup"), "worlds", "testbed.sdf"]
        ),
        description="Gazebo 월드 파일 (절대 경로 또는 Gazebo 컬렉션 파일명).",
    ))

    # MoveIt / 시간 인수
    declared_arguments.append(DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="모든 노드에 시뮬레이션 시간 사용.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "publish_robot_description_semantic", default_value="True",
        description="/robot_description_semantic 토픽에 SRDF 발행 여부.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "moveit_config_package", default_value="ur_moveit_config",
        description="MoveIt 설정 패키지 (OMPL / controller / servo YAML 제공).",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "srdf_package", default_value="ur_setup_bringup",
        description="SRDF xacro 파일이 있는 패키지.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "srdf_file", default_value="ur_robotiq.srdf.xacro",
        description="SRDF xacro 파일명 (srdf/ 디렉터리 상대 경로).",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "moveit_joint_limits_file", default_value="joint_limits.yaml",
        description="MoveIt 조인트 한계 파일명.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "warehouse_sqlite_path",
        default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
        description="MoveIt 웨어하우스 DB 경로.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_rviz", default_value="true",
        description="RViz 실행 여부.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_servo", default_value="true",
        description="MoveIt Servo 노드 실행 여부.",
    ))

    # 테스트베드 치수(페데스탈, 플레이트)는 런치 인수로 노출하지 않음.
    # 단일 진실 공급원(single source of truth):
    #   - 페데스탈/로봇 높이: urdf/ur16e_robotiq_2f85.urdf.xacro 의 xacro:arg 기본값
    #   - 플레이트 위치/크기: urdf/testbed.urdf.xacro 의 xacro:macro params 기본값
    # 치수 변경 시 해당 xacro 파일의 default 값만 수정하면 됨.

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
