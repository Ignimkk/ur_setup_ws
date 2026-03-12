# ur_setup_bringup

UR 로봇(UR5e / UR16e)과 Robotiq 2F-85 그리퍼를 Gazebo 시뮬레이션 환경에서 설정하고 스폰하기 위한 패키지입니다.
URDF, SRDF, MoveIt2 설정, 컨트롤러 설정, 시뮬레이션 월드 파일을 포함하며, 외부 패키지 의존 없이 독립적으로 시뮬레이션 환경을 구성합니다.

---

## 외부 의존 패키지

이 패키지를 사용하려면 아래 외부 패키지가 사전에 설치 및 빌드되어 있어야 합니다.

### 1. ros2_robotiq_gripper

Robotiq 2F-85 그리퍼의 URDF, ros2_control 하드웨어 인터페이스, 컨트롤러를 제공합니다.

- 저장소: https://github.com/PickNikRobotics/ros2_robotiq_gripper
- 필요 패키지: `robotiq_description`, `robotiq_controllers`, `robotiq_driver`

```bash
cd <your_ws>/src
git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
```

### 2. Universal Robots ROS2 Driver (ur_robot_driver)

실제 UR 로봇 하드웨어 연결 및 ros2_control 드라이버를 제공합니다. 시뮬레이션에서도 URDF 생성 및 컨트롤러 설정에 필요합니다.

- 저장소: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- 필요 패키지: `ur_robot_driver`, `ur_description`, `ur_moveit_config`

```bash
sudo apt install ros-${ROS_DISTRO}-ur-robot-driver
```

또는 소스 빌드:

```bash
cd <your_ws>/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
```

### 3. ur_simulation_gz (UR Gazebo 시뮬레이션)

Ignition Gazebo(ros_gz_sim) 기반 UR 시뮬레이션 플러그인 및 world 파일을 제공합니다. 이 패키지의 launch 파일은 내부적으로 `ur_simulation_gz`의 launch를 include하여 Gazebo를 실행합니다.

- 저장소: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation
- 필요 패키지: `ur_simulation_gz`

```bash
sudo apt install ros-${ROS_DISTRO}-ur-simulation-gz
```

또는 소스 빌드:

```bash
cd <your_ws>/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
```

### 4. RealSense ROS2 (realsense-ros)

Intel RealSense D435i 카메라 URDF 및 드라이버를 제공합니다. 카메라 브라켓 어댑터 URDF에서 카메라 메시 참조에 사용됩니다.

- 저장소: https://github.com/IntelRealSense/realsense-ros
- 필요 패키지: `realsense2_description`

```bash
sudo apt install ros-${ROS_DISTRO}-realsense2-description
```

---

## 패키지 구조

```
ur_setup_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── initial_positions.yaml               # UR 초기 관절 각도 (시뮬 스폰 시 적용)
│   ├── kinematics.yaml                      # MoveIt2용 kinematics solver 설정
│   ├── ur5e_robotiq_2f85_controllers.yaml   # UR5e + Robotiq 2F-85 ros2_control 컨트롤러 설정
│   └── ur5e/
│       ├── default_kinematics.yaml          # UR5e DH 파라미터 기반 기본 기구학
│       ├── joint_limits.yaml                # UR5e 관절 속도/가속도 제한
│       ├── physical_parameters.yaml         # UR5e 물리 파라미터 (질량, 관성 등)
│       └── visual_parameters.yaml           # UR5e 시각화 파라미터 (메시 경로 등)
├── launch/
│   ├── ur_sim_moveit_robotiq.launch.py      # UR5e + Robotiq + Gazebo + MoveIt2 통합 런치
│   └── ur_sim_moveit_robotiq_ur16e.launch.py # UR16e + Robotiq + 테스트베드 + MoveIt2 통합 런치
├── meshes/
│   ├── bracket/
│   │   └── ur_D435i_BRACKET.STL             # D435i 카메라 마운트 브라켓 메시
│   └── camera/
│       └── d435.dae                         # RealSense D435i 카메라 외형 메시
├── models/
│   └── pick_block/
│       └── model.sdf                        # Gazebo 스폰용 블록 오브젝트 SDF 모델
├── rviz/
│   └── robot_model.rviz                     # MoveIt2 RViz 설정 파일
├── srdf/
│   └── ur_robotiq.srdf.xacro               # UR + Robotiq 통합 MoveIt2 SRDF
│                                            # (planning group, end-effector, 충돌 제외 설정 포함)
├── urdf/
│   ├── ur5e_robotiq_2f85.urdf.xacro        # UR5e + Robotiq 2F-85 통합 URDF
│   ├── ur16e_robotiq_2f85.urdf.xacro       # UR16e + Robotiq 2F-85 통합 URDF
│   ├── testbed.urdf.xacro                  # 테스트베드(작업대) 환경 URDF
│   ├── camera_bracket_adapter.urdf.xacro   # D435i 카메라 + 브라켓 어댑터 링크 URDF
│   └── ur_to_robotiq_adapter.urdf.xacro    # UR tool0 -> Robotiq 마운트 어댑터 링크 URDF
└── worlds/
    └── testbed.sdf                          # 테스트베드 환경 Gazebo 월드 파일
```

---

## 주요 파일 설명

### URDF

| 파일 | 설명 |
|---|---|
| `ur5e_robotiq_2f85.urdf.xacro` | UR5e 본체에 어댑터 링크와 Robotiq 2F-85를 연결한 완전 통합 URDF. ros2_control 태그 포함. |
| `ur16e_robotiq_2f85.urdf.xacro` | UR16e 기반 동일 구조. 대형 페이로드 환경 대응. |
| `ur_to_robotiq_adapter.urdf.xacro` | UR `tool0` 프레임과 Robotiq `robotiq_85_base_link` 사이의 오프셋 어댑터 링크. |
| `camera_bracket_adapter.urdf.xacro` | D435i 카메라와 전용 브라켓을 로봇에 부착하기 위한 어댑터 링크. |
| `testbed.urdf.xacro` | 작업용 테이블/선반 구조물 URDF. UR16e 런치에서 world와 함께 스폰. |

### SRDF

| 파일 | 설명 |
|---|---|
| `ur_robotiq.srdf.xacro` | MoveIt2가 필요로 하는 로봇 시맨틱 정보 정의. planning group(`ur_manipulator`, `gripper`), end-effector, 홈 포지션, 자가충돌 제외 링크 쌍 포함. |

### Config

| 파일 | 설명 |
|---|---|
| `initial_positions.yaml` | Gazebo 시뮬 스폰 시 적용되는 UR 초기 관절 각도. |
| `kinematics.yaml` | MoveIt2 `move_group`이 사용하는 kinematics solver 종류 및 파라미터 (기본: KDL). |
| `ur5e_robotiq_2f85_controllers.yaml` | `joint_trajectory_controller`, `joint_state_broadcaster`, `robotiq_gripper_controller` 설정. ros2_control이 이 파일을 읽어 컨트롤러를 로드. |
| `ur5e/joint_limits.yaml` | UR5e 관절별 속도/가속도 제한값. MoveIt2 계획 시 참조. |
| `ur5e/default_kinematics.yaml` | UR5e DH 파라미터 기반 기본 기구학 파라미터. |

### Launch

| 파일 | 설명 |
|---|---|
| `ur_sim_moveit_robotiq.launch.py` | UR5e 전용 통합 런치. Gazebo, robot_state_publisher, ros2_control, MoveIt2(move_group), RViz를 한 번에 실행. |
| `ur_sim_moveit_robotiq_ur16e.launch.py` | UR16e 전용 통합 런치. 기본 월드로 testbed.sdf 사용. UR16e 설정은 `ur_description` 패키지 경로를 직접 참조. |

---

## 빌드

```bash
cd <your_ws>
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select ur_setup_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

의존 패키지를 포함하여 함께 빌드하는 경우:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 실행

### UR5e + Robotiq 2F-85 시뮬레이션

```bash
ros2 launch ur_setup_bringup ur_sim_moveit_robotiq.launch.py ur_type:=ur5e safety_limits:=true
```

- Ignition Gazebo GUI, UR5e + Robotiq 2F-85 모델이 스폰됩니다.
- `joint_state_broadcaster`, `joint_trajectory_controller`, `robotiq_gripper_controller`가 자동으로 활성화됩니다.
- MoveIt2 `move_group` 노드와 RViz가 함께 실행됩니다.

### UR16e + Robotiq 2F-85 + 테스트베드 시뮬레이션

```bash
ros2 launch ur_setup_bringup ur_sim_moveit_robotiq_ur16e.launch.py
```

- 기본 월드: `worlds/testbed.sdf`
- UR16e 기구학 설정은 `ur_description` 패키지의 `config/ur16e/` 경로를 직접 참조합니다.

---

## 주요 런치 인수

아래 인수는 두 launch 파일 공통으로 사용할 수 있습니다.

| 인수 | 기본값 | 설명 |
|---|---|---|
| `ur_type` | `ur5e` / `ur16e` | UR 로봇 모델 종류 |
| `safety_limits` | `true` | UR 안전 제한 적용 여부 |
| `safety_pos_margin` | `0.15` | 안전 위치 마진 [rad] |
| `safety_k_position` | `20` | 안전 위치 게인 |
| `prefix` | `""` | 관절/링크 이름 앞에 붙는 prefix (다중 로봇 구성 시 사용) |
| `gazebo_gui` | `true` | Gazebo GUI 실행 여부 |
| `world_file` | `empty.sdf` | Gazebo 월드 파일 경로 또는 파일명 |
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 여부 |
| `launch_rviz` | `true` | RViz 실행 여부 |
| `launch_servo` | `true` | MoveIt2 Servo 노드 실행 여부 |
| `srdf_package` | `ur_setup_bringup` | SRDF 파일이 위치한 패키지 이름 |
| `srdf_file` | `ur_robotiq.srdf.xacro` | 사용할 SRDF 파일명 |
| `moveit_joint_limits_file` | `joint_limits.yaml` | MoveIt2 관절 제한 파일명 |

---

## 그리퍼 제어

시뮬레이션 실행 후, `robotiq_gripper_controller`를 통해 GripperCommand 액션으로 그리퍼를 제어합니다.

```bash
# 컨트롤러 상태 확인
ros2 control list_controllers

# 그리퍼 열기
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 40.0}}"

# 그리퍼 닫기
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.7, max_effort: 40.0}}"
```

> 시뮬레이션에서는 `robotiq_activation_controller`를 사용하지 않습니다.
> 이 컨트롤러는 실제 하드웨어의 GPIO 기반 활성화 전용이며, Ignition Gazebo의 ros2_control 구성에서는 해당 GPIO 인터페이스가 제공되지 않습니다.

---

## 주의사항

- `ur_simulation_gz` 패키지가 없으면 Gazebo 런치가 실패합니다. 사전 설치를 확인하십시오.
- UR16e 런치는 `ur5e/` 설정 디렉토리를 사용하지 않고 `ur_description` 패키지의 `config/ur16e/`를 직접 참조합니다. 별도 복사가 필요하지 않습니다.
- MoveIt2 Planning Group 이름은 `ur_manipulator`(팔), `gripper`(그리퍼)로 고정되어 있습니다. 변경이 필요하면 `srdf/ur_robotiq.srdf.xacro`를 수정하십시오.
- 이 패키지는 설정 및 스폰 전용입니다. pick and place 등 태스크 노드는 별도 패키지에서 구현하고 이 패키지의 launch를 include하거나 의존성으로 추가하여 사용하십시오.
