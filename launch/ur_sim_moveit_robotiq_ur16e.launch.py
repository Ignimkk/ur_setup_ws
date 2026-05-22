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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    spawn_alphabet = LaunchConfiguration("spawn_alphabet")

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
    # Gazebo 리소스 경로 설정
    # Ignition은 model://패키지명/... URI를 해석할 때 IGN_GAZEBO_RESOURCE_PATH에서
    # "패키지명" 디렉토리를 탐색함. 따라서 share/ 디렉토리(패키지 디렉토리의 부모)를 등록해야 함.
    # PathJoinSubstitution([..., ".."]) 은 경로에 리터럴 ".."가 남아 Ignition이 해석 불가하므로
    # OpaqueFunction 컨텍스트에서 os.path.dirname + get_package_share_directory 로 실제 경로를 얻음.
    # ------------------------------------------------------------------ #
    robotiq_share_parent = os.path.dirname(
        get_package_share_directory("robotiq_description")
    )
    ur_setup_bringup_share_parent = os.path.dirname(
        get_package_share_directory("ur_setup_bringup")
    )
    realsense_share_parent = os.path.dirname(
        get_package_share_directory("realsense2_description")
    )
    resource_paths = ":".join([
        robotiq_share_parent,
        ur_setup_bringup_share_parent,
        realsense_share_parent,
    ])
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            resource_paths,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            resource_paths,
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
    # Gazebo 에 pick-and-place 대상 alphabet mesh 스폰
    # 각 알파벳은 URDF 고정 링크가 아니라 별도 동적 SDF 모델이다.
    # plate: center=(0.60, 0.25), size=0.6 x 0.6, top z=0.92.
    # 9개 대상(A,B,D,E,E,G,I,N,R)을 3x3 격자로 배치한다.
    # STL 단위는 mm이고 각 SDF에서 scale=0.001을 적용한다.
    # ------------------------------------------------------------------ #
    # EDGE BRAIN 순서: E1, D, G, E2, B, R, A, I, N
    # 리스트 순서는 EDGE BRAIN sequence 와 동일하게 맞추어, alphabet_count 로
    # 앞에서부터 자르면 자연스럽게 "테스트용 첫 N 글자" 가 된다.
    alphabet_specs_all = [
        ("alphabet_E1", "alphabet_E", "0.42", "0.25", "0.92"),  # EDGE BRAIN[0]
        ("alphabet_D",  "alphabet_D", "0.78", "0.43", "0.92"),  # EDGE BRAIN[1]
        ("alphabet_G",  "alphabet_G", "0.78", "0.25", "0.92"),  # EDGE BRAIN[2]
        ("alphabet_E2", "alphabet_E", "0.60", "0.25", "0.92"),  # EDGE BRAIN[3]
        ("alphabet_B",  "alphabet_B", "0.60", "0.43", "0.92"),  # EDGE BRAIN[4]
        ("alphabet_R",  "alphabet_R", "0.78", "0.07", "0.92"),  # EDGE BRAIN[5]
        ("alphabet_A",  "alphabet_A", "0.42", "0.43", "0.92"),  # EDGE BRAIN[6]
        ("alphabet_I",  "alphabet_I", "0.42", "0.07", "0.92"),  # EDGE BRAIN[7]
        ("alphabet_N",  "alphabet_N", "0.60", "0.07", "0.92"),  # EDGE BRAIN[8]
    ]
    alphabet_count = int(LaunchConfiguration("alphabet_count").perform(context))
    alphabet_count = max(0, min(alphabet_count, len(alphabet_specs_all)))
    alphabet_specs = alphabet_specs_all[:alphabet_count]
    alphabet_spawners = [
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-file",
                PathJoinSubstitution([
                    FindPackageShare(description_package),
                    "models",
                    model_dir,
                    "model.sdf",
                ]),
                "-name", name,
                "-x", x,
                "-y", y,
                "-z", z,
            ],
        )
        for name, model_dir, x, y, z in alphabet_specs
    ]
    # 로봇 스폰(gz_spawn_entity) 프로세스가 종료된 후(=시뮬레이터에 로봇 엔티티가
    # 등록되어 첫 frame 이 안정화된 시점) 3 초 뒤에 알파벳을 마지막으로 스폰한다.
    #
    # 배경: STL collision 메시 9개를 로봇과 동시에 등록하면 초기 contact 해석
    #       비용으로 RTF 가 급락하여 controller / action 응답이 지연된다.
    #       로봇이 먼저 안착한 뒤 알파벳을 추가하면 초기 부하 spike 를 회피한다.
    # DetachableJoint plugin 의 attachRequested 기본값이 true 라서
    # alphabet 이 spawn 되는 순간 자동으로 wrist_3_link 에 attach 되어
    # 9 개 letter 가 모두 robot 에 끌려가 떨어지는 문제가 있음.
    # 대응: alphabet spawn 전에 detach 메시지를 미리 publish 하여 plugin 의
    # detachRequested 를 true 로 만들어 둠. plugin source (PreUpdate) 가
    # 한 tick 안에 attach→detach 분기를 순차 실행하므로, alphabet 이 발견되어
    # attach 가 일어나는 같은 tick 에 즉시 detach 가 처리되어 letter 가
    # 원위치에 그대로 머무름.
    # ign cli 로 ignition.msgs.Empty 를 여러 번 발행 (subscriber timing 안전).
    detach_pre_spam = ExecuteProcess(
        cmd=[
            "bash", "-c",
            # 0.2s 간격으로 15 번 = 약 3 초 동안 detach 트리거 유지.
            # 이 윈도우가 alphabet spawn (가변 timing) 을 충분히 덮음.
            "for i in $(seq 1 15); do "
            "ign topic -t /grasp/release_all "
            "-m ignition.msgs.Empty -p '' >/dev/null 2>&1; "
            "sleep 0.2; done"
        ],
        output="screen",
    )

    alphabet_spawn_start = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                # 1) 즉시 detach spam 시작 (백그라운드, ~3s 동안 발행)
                detach_pre_spam,
                # 2) 5s 뒤 alphabet spawn (detach spam 이 spawn 시점을 덮도록)
                TimerAction(
                    period=10.0,
                    actions=alphabet_spawners,
                    condition=IfCondition(spawn_alphabet),
                ),
            ],
        )
    )

    # ------------------------------------------------------------------ #
    # /clock + 카메라 토픽 브리지 (Ignition → ROS 2)
    # 브리지 형식: <ignition_topic>@<ros2_msg_type>[<ignition_msg_type>
    #   [ : Ignition → ROS2 단방향
    # ------------------------------------------------------------------ #
    # prefix 기본값이 '""'(쌍따옴표 포함 문자열)이므로 strip('"')으로 제거해야 실제 빈 문자열을 얻음.
    # 예: '""'.strip('"') = ''  /  'robot01_'.strip('"') = 'robot01_'
    camera_prefix = prefix.perform(context).strip('"')

    # DetachableJoint 플러그인은 ignition::transport 로 attach/detach 토픽을
    # 구독하므로, dispatcher / pick_place_node 가 publish 하는 ROS 토픽을
    # Ignition 으로 흘려보내려면 ros_gz_bridge 에 ROS→Ign 매핑(`]`)이 필요.
    alphabet_names = [
        "alphabet_E1", "alphabet_D",  "alphabet_G",
        "alphabet_E2", "alphabet_B",  "alphabet_R",
        "alphabet_A",  "alphabet_I",  "alphabet_N",
    ]
    attach_bridge_args = [
        f"/grasp/attach/{n}@std_msgs/msg/Empty]ignition.msgs.Empty"
        for n in alphabet_names
    ]
    # /grasp/release_all 은 공용 detach 트리거 (ROS → Ign).
    # /grasp/state/<name> 은 plugin 의 상태 변화 알림 (Ign → ROS, 디버깅용).
    state_bridge_args = [
        f"/grasp/state/{n}@std_msgs/msg/String[ignition.msgs.StringMsg"
        for n in alphabet_names
    ]

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # 컬러 이미지
            f"/{camera_prefix}camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            # 뎁스 이미지
            f"/{camera_prefix}camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            # 카메라 정보 (rgbd_camera는 camera_info를 단 1개만 발행함)
            f"/{camera_prefix}camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # 포인트 클라우드
            f"/{camera_prefix}camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # DetachableJoint attach 트리거 (ROS → Ign)
            *attach_bridge_args,
            # DetachableJoint 공용 detach 트리거 (ROS → Ign)
            "/grasp/release_all@std_msgs/msg/Empty]ignition.msgs.Empty",
            # DetachableJoint 상태 알림 (Ign → ROS, 디버깅)
            *state_bridge_args,
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
        # 알파벳 스폰은 로봇 + 컨트롤러 + MoveIt 가 모두 시작된 뒤
        # 마지막에 트리거되도록 리스트 끝에 배치.
        alphabet_spawn_start,
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
    declared_arguments.append(DeclareLaunchArgument(
        "spawn_alphabet", default_value="true",
        description="시작 시 plate 위 alphabet pick 대상들을 Gazebo에 스폰할지 여부.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "alphabet_count", default_value="2",
        description=(
            "EDGE BRAIN 시퀀스의 앞에서부터 몇 개의 알파벳을 스폰할지 (0~9). "
            "테스트 시에는 2~3 권장. 전체 데모는 9 로 설정."
        ),
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
