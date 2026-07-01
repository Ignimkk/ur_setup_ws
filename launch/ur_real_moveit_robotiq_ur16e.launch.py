"""
ur_real_moveit_robotiq_ur16e.launch.py

실물 UR16e + Robotiq 2F-85 (+ pedestal/plate/pallet 정적 충돌체) bring-up.

대응 sim 런치: ur_sim_moveit_robotiq_ur16e.launch.py
공통:
  - URDF macro / SRDF / MoveIt 설정은 동일 구조 (ur_manipulator planning group,
    base_link 기준 IK, robotiq_85_left_knuckle_joint 그리퍼 등)
차이:
  - Gazebo / gz_ros2_control 미사용
  - use_sim_time = false
  - 실물 UR controller_manager 는 ur_robot_driver/ur_ros2_control_node 가 spawn
  - 실물 Robotiq 2F-85 는 같은 controller_manager 안의 robotiq_driver hardware
    interface (URDF macro 가 robotiq_driver/RobotiqGripperHardwareInterface 를 emit)
  - MoveIt 측 default controller 는 scaled_joint_trajectory_controller
  - **자동 모션 방지**: scaled_joint_trajectory_controller 가
    activate_joint_controller=false 일 때 spawner --inactive 로 로드만 됨.
    플래닝(MoveIt) 후 실제 실행 직전 사용자가 명시적으로 controller 활성화 필요:
      ros2 control switch_controllers \
        --activate scaled_joint_trajectory_controller

사용 예:
  # Robotiq가 UR Tool RS485에 연결된 경우
  ros2 launch ur_setup_bringup ur_real_moveit_robotiq_ur16e.launch.py \
    robot_ip:=192.168.56.101 \
    gripper_on_tool:=true \
    launch_rviz:=false

  # 실행 권한 부여 (자동 활성화):
  ros2 launch ur_setup_bringup ur_real_moveit_robotiq_ur16e.launch.py \
    robot_ip:=192.168.56.101 activate_joint_controller:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    # ---------------- 실행 분기 ----------------
    launch_control = LaunchConfiguration("launch_control")
    launch_moveit  = LaunchConfiguration("launch_moveit")

    # ---------------- UR / 안전 인수 ----------------
    ur_type           = LaunchConfiguration("ur_type")
    robot_ip          = LaunchConfiguration("robot_ip")
    safety_limits     = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix            = LaunchConfiguration("prefix")
    headless_mode     = LaunchConfiguration("headless_mode")

    # ---------------- 그리퍼 인수 ----------------
    # gripper_on_tool=true  → Robotiq 가 UR16e Tool I/O 에 연결됨.
    #   PC 측 device 는 tool_communication.py 가 만드는 /tmp/ttyUR (PTY).
    # gripper_on_tool=false → PC USB-RS485 어댑터 직결. gripper_com_port 인수 사용.
    gripper_on_tool   = LaunchConfiguration("gripper_on_tool")
    gripper_com_port_arg  = LaunchConfiguration("gripper_com_port")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    enable_direct_robotiq_control = LaunchConfiguration("enable_direct_robotiq_control")
    tool_tcp_port     = LaunchConfiguration("tool_tcp_port")
    tool_device_name  = LaunchConfiguration("tool_device_name")
    tool_voltage      = LaunchConfiguration("tool_voltage")
    tool_parity       = LaunchConfiguration("tool_parity")
    tool_baud_rate    = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits    = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")

    # use_tool_communication=true 일 때만 UR tool RS485 PTY(/tmp/ttyUR)를 사용한다.
    # 기본 실물 구성은 URCap/teach pendant와 동일한 digital_out[1] 제어이므로
    # tool communication과 Robotiq direct Modbus hardware를 켜지 않는다.
    gripper_com_port_effective = PythonExpression([
        "'", tool_device_name, "' if '", use_tool_communication, "' == 'true' else '",
        gripper_com_port_arg, "'"
    ])
    robotiq_use_fake_hardware = PythonExpression([
        "'false' if '", enable_direct_robotiq_control, "' == 'true' else 'true'"
    ])

    # ---------------- 테스트베드 캘리브레이션 인수 ----------------
    pedestal_x        = LaunchConfiguration("pedestal_x")
    pedestal_y        = LaunchConfiguration("pedestal_y")
    pedestal_z        = LaunchConfiguration("pedestal_z")
    plate_x           = LaunchConfiguration("plate_x")
    plate_y           = LaunchConfiguration("plate_y")
    pallet_x          = LaunchConfiguration("pallet_x")
    pallet_y          = LaunchConfiguration("pallet_y")

    # ---------------- 컨트롤러 인수 ----------------
    initial_joint_controller  = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_dashboard_client   = LaunchConfiguration("launch_dashboard_client")

    # ---------------- MoveIt / RViz 인수 ----------------
    launch_rviz                       = LaunchConfiguration("launch_rviz")
    launch_servo                      = LaunchConfiguration("launch_servo")
    moveit_joint_limits_file          = LaunchConfiguration("moveit_joint_limits_file")
    warehouse_sqlite_path             = LaunchConfiguration("warehouse_sqlite_path")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    srdf_package          = LaunchConfiguration("srdf_package")
    srdf_file             = LaunchConfiguration("srdf_file")

    # ---------------- 패키지 / 파일 ----------------
    description_package   = "ur_setup_bringup"
    description_file      = "ur16e_robotiq_2f85_real.urdf.xacro"
    runtime_config_package = "ur_setup_bringup"
    controllers_file      = PythonExpression([
        "'ur16e_robotiq_2f85_real_direct_controllers.yaml' if '",
        enable_direct_robotiq_control,
        "' == 'true' else 'ur16e_robotiq_2f85_real_controllers.yaml'"
    ])

    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

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

    # ur_robot_driver 가 사용하는 ur16e 실시간 주기(=500 Hz) 파일
    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", "ur16e_update_rate.yaml"]
    )

    # External Control URScript / RTDE recipe
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    # ---------------- robot_description ----------------
    robot_description_content = Command([
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
        " tf_prefix:=",             prefix,
        " sim_gazebo:=false",
        " sim_ignition:=false",
        " use_fake_hardware:=false",
        " headless_mode:=",         headless_mode,
        " robot_ip:=",              robot_ip,
        " script_filename:=",       script_filename,
        " input_recipe_filename:=", input_recipe_filename,
        " output_recipe_filename:=", output_recipe_filename,
        # Robotiq 시리얼 경로 — use_tool_communication=true 면 /tmp/ttyUR (PTY).
        " gripper_com_port:=",      gripper_com_port_effective,
        " include_robotiq_ros2_control:=true",
        " robotiq_use_fake_hardware:=", robotiq_use_fake_hardware,
        # UR tool I/O 패스스루
        " use_tool_communication:=", use_tool_communication,
        " tool_voltage:=",          tool_voltage,
        " tool_parity:=",           tool_parity,
        " tool_baud_rate:=",        tool_baud_rate,
        " tool_stop_bits:=",        tool_stop_bits,
        " tool_rx_idle_chars:=",    tool_rx_idle_chars,
        " tool_tx_idle_chars:=",    tool_tx_idle_chars,
        " tool_device_name:=",      tool_device_name,
        " tool_tcp_port:=",         tool_tcp_port,
        " pedestal_x:=",            pedestal_x,
        " pedestal_y:=",            pedestal_y,
        " pedestal_z:=",            pedestal_z,
        " plate_x:=",               plate_x,
        " plate_y:=",               plate_y,
        " pallet_x:=",              pallet_x,
        " pallet_y:=",              pallet_y,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ---------------- UR Tool I/O ↔ PC PTY 터널 ----------------
    # 공식 ur_robot_driver 방식:
    #   UR 컨트롤러 TCP port 54321 ↔ /tmp/ttyUR(PTY)
    #
    # 중요:
    #   tool_communication.py는 일반 CLI positional argument가 아니라 ROS 2
    #   parameter(robot_ip, tcp_port, device_name)를 받는 노드로 실행해야 한다.
    #   gripper_on_tool=false이면 PC USB-RS485 직결이므로 실행하지 않는다.
    tool_communication_node = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        condition=IfCondition(PythonExpression([
            "'", launch_control, "' == 'true' and '",
            use_tool_communication, "' == 'true'"
        ])),
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": tool_device_name,
            }
        ],
    )

    # ---------------- 실물 ros2_control 노드 ----------------
    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        condition=IfCondition(launch_control),
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(controllers_file_path, allow_substs=True),
        ],
        output="screen",
    )

    # ---------------- robot_state_publisher ----------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=IfCondition(launch_control),
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],
    )

    # ---------------- UR Dashboard / IO 보조 노드 ----------------
    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
        condition=IfCondition(PythonExpression([
            "'", launch_control, "' == 'true' and '",
            launch_dashboard_client, "' == 'true'"
        ])),
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        condition=IfCondition(launch_control),
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    controller_stopper = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        condition=IfCondition(launch_control),
        output="screen",
        emulate_tty=True,
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {"consistent_controllers": [
                "io_and_status_controller",
                "force_torque_sensor_broadcaster",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster",
            ]},
        ],
    )

    # ---------------- controller spawner ----------------
    def spawner(name, *extra_args, active=True):
        args = [name, "--controller-manager", "/controller_manager"]
        if not active:
            args.append("--inactive")
        args.extend(extra_args)
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            condition=IfCondition(launch_control),
        )

    joint_state_broadcaster_spawner       = spawner("joint_state_broadcaster")
    io_and_status_controller_spawner      = spawner("io_and_status_controller")
    speed_scaling_state_broadcaster_spawner = spawner("speed_scaling_state_broadcaster")
    force_torque_sensor_broadcaster_spawner = spawner("force_torque_sensor_broadcaster")

    # UR trajectory controller — activate_joint_controller=true 면 active,
    # false 면 inactive 로 로드 (자동 모션 방지).
    initial_joint_controller_spawner_active = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression([
            "'", launch_control, "' == 'true' and '",
            activate_joint_controller, "' == 'true'"
        ])),
    )
    initial_joint_controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--inactive"],
        condition=IfCondition(PythonExpression([
            "'", launch_control, "' == 'true' and '",
            activate_joint_controller, "' != 'true'"
        ])),
    )

    # Robotiq controller는 여기서 자동 spawn하지 않는다.
    # 이유:
    #   gripper_on_tool=true일 때 /tmp/ttyUR가 생성되기 전에 그리퍼 hardware가
    #   configure되면 controller_manager 전체가 종료될 수 있다.
    #
    # controllers YAML의 hardware_components_initial_state에서
    # robotiq_2f_85를 unconfigured로 시작한 뒤, /tmp/ttyUR 생성 확인 후
    # 아래 순서로 수동 활성화한다.
    #
    #   ros2 control set_hardware_component_state robotiq_2f_85 inactive
    #   ros2 control set_hardware_component_state robotiq_2f_85 active
    #   ros2 run controller_manager spawner robotiq_activation_controller \
    #     -c /controller_manager
    #   ros2 run controller_manager spawner robotiq_gripper_controller \
    #     -c /controller_manager

    # ---------------- MoveIt ----------------
    # SRDF (sim 과 동일 파일 — 링크/조인트 토폴로지가 같음)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(srdf_package), "srdf", srdf_file]),
        " name:=ur",
        " prefix:=", prefix,
    ])
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

    # OMPL
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

    # 실물: MoveIt 의 default controller 를 scaled_joint_trajectory_controller 로 유지.
    # ur_moveit_config/controllers.yaml 의 기본값이 이미 scaled.
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = True
    controllers_yaml["joint_trajectory_controller"]["default"] = False

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
            {"use_sim_time": False},
            warehouse_ros_config,
        ],
    )

    # ---------------- RViz ----------------
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
            {"use_sim_time": False},
        ],
    )

    # ---------------- MoveIt Servo (선택) ----------------
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
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # ---------------- 기동 순서 ----------------
    # ros2_control 노드와 robot_state_publisher 가 먼저 뜨고,
    # broadcaster 들 → trajectory / gripper controller 순서로 spawn 후 MoveIt 시작.
    delayed_controllers_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                io_and_status_controller_spawner,
                speed_scaling_state_broadcaster_spawner,
                force_torque_sensor_broadcaster_spawner,
                initial_joint_controller_spawner_active,
                initial_joint_controller_spawner_inactive,
            ],
        )
    )

    moveit_start = TimerAction(
        period=5.0,
        actions=[move_group_node, rviz_node, servo_node],
        condition=IfCondition(launch_moveit),
    )

    return [
        tool_communication_node,
        ur_control_node,
        robot_state_publisher_node,
        dashboard_client_node,
        urscript_interface,
        controller_stopper,
        joint_state_broadcaster_spawner,
        delayed_controllers_after_jsb,
        moveit_start,
    ]


def generate_launch_description():
    declared_arguments = []

    # ---------------- 실행 분기 ----------------
    declared_arguments.append(DeclareLaunchArgument(
        "launch_control",
        default_value="false",
        description=(
            "true: ur_robot_driver ros2_control 및 controller spawner 실행. "
            "false(기본): MoveIt/RViz만 실행."
        )))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_moveit",
        default_value="true",
        description="true: move_group/RViz/Servo 실행."))

    # ---------------- UR / 안전 ----------------
    declared_arguments.append(DeclareLaunchArgument(
        "ur_type", default_value="ur16e",
        description="UR 로봇 모델."))
    declared_arguments.append(DeclareLaunchArgument(
        "robot_ip", default_value="192.168.56.101",
        description="실물 UR 컨트롤박스 IP."))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_limits", default_value="true",
        description="안전 한계 컨트롤러 활성화 여부."))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_k_position", default_value="20"))
    declared_arguments.append(DeclareLaunchArgument(
        "prefix", default_value="",
        description="조인트/링크 이름 prefix. 기본 빈 문자열."))
    declared_arguments.append(DeclareLaunchArgument(
        "headless_mode", default_value="false",
        description="UR Polyscope 없이 외부 제어. Polyscope External Control URCap "
                    "사용 시 false. headless 모드는 polyscope 5.10+ 필요."))

    # ---------------- 컨트롤러 ----------------
    declared_arguments.append(DeclareLaunchArgument(
        "initial_joint_controller",
        default_value="scaled_joint_trajectory_controller",
        description="MoveIt 이 실행에 사용할 UR trajectory controller."))
    declared_arguments.append(DeclareLaunchArgument(
        "activate_joint_controller", default_value="false",
        description=(
            "true: UR trajectory controller 를 active 로 로드 (즉시 trajectory 실행 가능). "
            "false(기본): --inactive 로 로드. 자동 모션 방지. "
            "planning 검증 후 사용자가 다음 명령으로 활성화:\n"
            "  ros2 control switch_controllers --activate scaled_joint_trajectory_controller"
        )))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_dashboard_client", default_value="true",
        description="UR Dashboard 클라이언트 노드 실행 여부 (로봇 power on/off, brake release 등)."))

    # ---------------- 그리퍼 ----------------
    declared_arguments.append(DeclareLaunchArgument(
        "gripper_on_tool", default_value="true",
        choices=["true", "false"],
        description=(
            "물리 배선 설명용 인수. true: Robotiq 가 UR16e Tool 커넥터에 연결됨. "
            "false: PC USB-RS485 어댑터 직결. gripper_com_port 인수로 device 지정."
        )))
    declared_arguments.append(DeclareLaunchArgument(
        "use_tool_communication", default_value="false",
        choices=["true", "false"],
        description=(
            "true일 때만 ur_robot_driver/tool_communication.py를 실행하고 "
            "Robotiq COM_port를 /tmp/ttyUR로 설정. 기본 false: URCap에서 검증된 "
            "digital_out[1] 제어 경로를 사용하며 /tmp/ttyUR에 의존하지 않음."
        )))
    declared_arguments.append(DeclareLaunchArgument(
        "enable_direct_robotiq_control", default_value="false",
        choices=["true", "false"],
        description=(
            "true: robotiq_driver/RobotiqGripperHardwareInterface를 사용해 직접 "
            "Modbus RTU 제어를 시도. false(기본): gripper ros2_control hardware는 "
            "fake로 유지하고 실제 gripper는 UR IO 서비스로 제어."
        )))
    declared_arguments.append(DeclareLaunchArgument(
        "gripper_com_port", default_value="/dev/ttyUSB0",
        description="use_tool_communication=false 이면서 direct Robotiq를 쓸 때의 USB-RS485 경로."))

    # ---------------- UR Tool I/O 통신 파라미터 ----------------
    # 모두 Robotiq 2F-85 권장 RS485 설정. use_tool_communication=true 일 때만 의미 있음.
    declared_arguments.append(DeclareLaunchArgument(
        "tool_voltage", default_value="24",
        description="Tool flange 출력 전압 [V]. Robotiq 2F-85 = 24V."))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_parity", default_value="0",
        description="0=None, 1=Odd, 2=Even"))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_baud_rate", default_value="115200"))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_stop_bits", default_value="1"))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_rx_idle_chars", default_value="1.5"))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_tx_idle_chars", default_value="3.5"))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_device_name", default_value="/tmp/ttyUR",
        description="tool_communication.py 가 생성할 PTY 경로. "
                    "Robotiq driver 의 COM_port 로도 사용됨."))
    declared_arguments.append(DeclareLaunchArgument(
        "tool_tcp_port", default_value="54321",
        description="UR 컨트롤러의 RS485 패스스루 TCP 포트."))

    # ---------------- 테스트베드 캘리브레이션 ----------------
    declared_arguments.append(DeclareLaunchArgument("pedestal_x", default_value="2.0"))
    declared_arguments.append(DeclareLaunchArgument("pedestal_y", default_value="0.8"))
    declared_arguments.append(DeclareLaunchArgument("pedestal_z", default_value="0.9",
        description="페데스탈 상단까지의 높이 [m]. 로봇 base_link 가 z=pedestal_z 에 마운트됨."))
    declared_arguments.append(DeclareLaunchArgument("plate_x", default_value="0.60",
        description="좌측 plate 중심 X [robot base 프레임]."))
    declared_arguments.append(DeclareLaunchArgument("plate_y", default_value="0.25"))
    declared_arguments.append(DeclareLaunchArgument("pallet_x", default_value="-0.60"))
    declared_arguments.append(DeclareLaunchArgument("pallet_y", default_value="0.35"))

    # ---------------- MoveIt / RViz ----------------
    declared_arguments.append(DeclareLaunchArgument(
        "publish_robot_description_semantic", default_value="True"))
    declared_arguments.append(DeclareLaunchArgument(
        "moveit_config_package", default_value="ur_moveit_config"))
    declared_arguments.append(DeclareLaunchArgument(
        "srdf_package", default_value="ur_setup_bringup"))
    declared_arguments.append(DeclareLaunchArgument(
        "srdf_file", default_value="ur_robotiq.srdf.xacro"))
    declared_arguments.append(DeclareLaunchArgument(
        "moveit_joint_limits_file", default_value="joint_limits.yaml"))
    declared_arguments.append(DeclareLaunchArgument(
        "warehouse_sqlite_path",
        default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_rviz", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_servo", default_value="false",
        description="MoveIt Servo. 실물에서는 기본 비활성화."))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
