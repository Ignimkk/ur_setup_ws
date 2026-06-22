# UR16e + Robotiq 2F-85 실물 전용 설치 및 실행 가이드

> **Gazebo-free 구성:** 이 문서는 Gazebo 시뮬레이션 패키지를 설치하지 않는 실물 전용 PC 구성을 기준으로 합니다.  
> **안전 주의:** 최초 실물 테스트에서는 Teach Pendant의 속도 슬라이더를 10~20%로 낮추고, RViz에서 반드시 `Plan Only`로 궤적과 충돌을 확인한 뒤 실행하세요.

## 0. 사전 조건

- Ubuntu 22.04 Jammy
- ROS 2 Humble Desktop
- UR16e
- Robotiq 2F-85
- USB-RS485 어댑터
  - 일반적인 장치 경로: `/dev/ttyUSB0`
- UR Polyscope 5.10 이상
- External Control URCap 지원 환경

---

## 1. 시스템 패키지 설치

Gazebo 관련 패키지는 설치하지 않습니다.

```bash
sudo apt update

sudo apt install -y \
  ros-humble-ur \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-warehouse-ros-sqlite \
  ros-humble-realsense2-description \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  git
```

`rosdep` 초기화:

```bash
sudo rosdep init || true
rosdep update
```

다음 Gazebo 패키지는 설치 대상에서 제외합니다.

```text
ros-humble-ros-gz
ros-humble-ros-gz-bridge
```

실물 전용 PC에서는 Gazebo 관련 패키지를 제외하여 설치 용량을 줄일 수 있습니다.

---

## 2. 워크스페이스 클론

```bash
mkdir -p ~/ur_setup_ws/src
cd ~/ur_setup_ws/src
```

프로젝트 패키지:

```bash
git clone -b main https://github.com/Ignimkk/ur_setup_ws.git
git clone -b main https://github.com/Ignimkk/pick_place_module.git
```

외부 의존 패키지:

```bash
git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
git clone -b ros2 https://github.com/PickNikRobotics/bio_ik.git
git clone -b master https://github.com/RoverRobotics-forks/serial-ros2.git
```

실물 전용 PC에서는 다음 Gazebo 패키지를 클론하지 않습니다.

```text
Universal_Robots_ROS2_GZ_Simulation
```

기존 워크스페이스에 이미 클론되어 있다면 빌드 대상에서 제외합니다.

```bash
touch ~/ur_setup_ws/src/Universal_Robots_ROS2_GZ_Simulation/COLCON_IGNORE
```

해당 디렉터리가 없으면 위 명령은 실행하지 않아도 됩니다.

`path_planner_benchmark`도 실물 구동에 필요하지 않다면 제외할 수 있습니다.

---

## 3. 의존성 설치

ROS 2 Humble 환경을 먼저 적용합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ur_setup_ws
```

Gazebo 관련 rosdep 키와 소스에서 직접 빌드할 `serial` 키를 제외합니다.

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --skip-keys "serial ros_gz_sim ros_gz_bridge"
```

### 선택: `package.xml` 정리

실물 전용 PC에서만 해당 저장소를 사용할 경우 `ur_setup_bringup/package.xml`의 Gazebo 의존성을 주석 처리할 수 있습니다.

```xml
<!-- 실물 전용 PC에서는 아래 Gazebo 의존성을 주석 처리할 수 있음 -->
<!-- <exec_depend>ros_gz_sim</exec_depend> -->
<!-- <exec_depend>ros_gz_bridge</exec_depend> -->
```

시뮬레이션 PC와 실물 PC가 동일한 저장소를 사용한다면 `package.xml`은 유지하고 `--skip-keys`를 사용하는 편이 안전합니다.

---

## 4. 빌드

`serial` 정적 라이브러리를 `robotiq_driver` 공유 라이브러리에 연결할 수 있도록 PIC 옵션을 적용합니다.

```bash
cd ~/ur_setup_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
```

환경 적용:

```bash
source install/setup.bash
echo "source ~/ur_setup_ws/install/setup.bash" >> ~/.bashrc
```

패키지 설치 확인:

```bash
ros2 pkg prefix ur_setup_bringup
ros2 pkg prefix robotiq_driver
ros2 pkg prefix pick_place_module
```

실물 launch가 Gazebo 없이 로드되는지 확인합니다.

```bash
ros2 launch ur_setup_bringup \
  ur_real_moveit_robotiq_ur16e.launch.py \
  --show-args
```

인수 목록이 정상적으로 출력되면 실물 launch import가 통과한 것입니다.

다음 오류가 발생하면 실물 launch가 Gazebo 모듈을 직접 import하는지 확인해야 합니다.

```text
ModuleNotFoundError: No module named 'ros_gz_sim'
```

시뮬레이션 launch를 잘못 실행한 경우에는 실물 launch 자체와 무관할 수 있습니다.

---

## 5. 네트워크 설정

### 5.1 UR 컨트롤박스

Teach Pendant에서 다음 메뉴로 이동합니다.

```text
Setup Robot → Network → Static Address
```

설정값:

```text
IP Address : 192.168.56.101
Subnet Mask: 255.255.255.0
Gateway    : 비움
```

### 5.2 PC 유선 NIC

UR과 연결된 Ethernet NIC에 같은 서브넷의 다른 IP를 할당합니다.

NIC 이름 확인:

```bash
ip link
```

임시 설정:

```bash
sudo ip addr add 192.168.56.10/24 dev <NIC명>
sudo ip link set <NIC명> up
```

예:

```bash
sudo ip addr add 192.168.56.10/24 dev enp3s0
sudo ip link set enp3s0 up
```

연결 확인:

```bash
ping -c 3 192.168.56.101
```

영구 설정:

```text
Ubuntu Settings → Network → Wired → IPv4 → Manual
```

입력값:

```text
Address: 192.168.56.10
Netmask: 255.255.255.0
Gateway: 비움
```

### 5.3 방화벽

UFW를 사용 중이면 UR 전용 서브넷을 허용합니다.

```bash
sudo ufw allow from 192.168.56.0/24
sudo ufw status
```

---

## 6. External Control URCap 설치

URCap 설치는 최초 한 번만 수행합니다.

URCap 파일 위치 확인:

```bash
ls /opt/ros/humble/share/ur_robot_driver/resources/ | grep -i urcap
```

예:

```text
externalcontrol-1.0.5.urcap
```

설치 순서:

1. URCap 파일을 USB에 복사합니다.
2. USB를 Teach Pendant에 연결합니다.
3. 다음 메뉴로 이동합니다.

```text
Setup → URCaps → +
```

4. `externalcontrol-*.urcap` 파일을 선택합니다.
5. `Install`을 누릅니다.
6. UR 컨트롤러를 재시작합니다.

재부팅 후 다음 메뉴로 이동합니다.

```text
Installation → URCaps → External Control
```

설정값:

```text
Host IP    : 192.168.56.10
Custom Port: 50002
```

`Host IP`는 UR 로봇 IP가 아니라 ROS 2 PC의 유선 NIC IP입니다.

프로그램 생성:

1. `Program` 메뉴로 이동합니다.
2. `URCaps → External Control` 노드를 추가합니다.
3. External Control 노드가 포함된 프로그램을 저장합니다.
4. 실물 제어 시마다 해당 프로그램에서 `Play`를 누릅니다.

---

## 7. Robotiq USB-RS485 권한 설정

어댑터 정보 확인:

```bash
udevadm info -a -n /dev/ttyUSB0 \
  | grep -E "idVendor|idProduct" \
  | head -2
```

FTDI 어댑터의 일반적인 값:

```text
idVendor : 0403
idProduct: 6001
```

udev rule 생성:

```bash
sudo tee /etc/udev/rules.d/99-robotiq.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="robotiq", MODE="0666"
EOF
```

적용:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

확인:

```bash
ls -l /dev/robotiq
```

이후 launch에서는 다음 포트를 사용합니다.

```text
gripper_com_port:=/dev/robotiq
```

어댑터가 여러 개라면 udev rule에 장치 시리얼 번호 조건을 추가합니다.

```text
ATTRS{serial}=="장치_시리얼번호"
```

필요한 경우 사용자를 `dialout` 그룹에 추가합니다.

```bash
sudo usermod -aG dialout $USER
```

적용을 위해 로그아웃 후 다시 로그인합니다.

---

## 8. 매 세션 기동 순서

### 8.1 UR 로봇

1. UR 컨트롤박스 전원 ON
2. Teach Pendant에서 `Initialize`
3. `Power ON`
4. `Brake Release`
5. External Control 프로그램 열기
6. `Play` 실행

### 8.2 ROS 2 드라이버, MoveIt, RViz

터미널 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ur_setup_ws/install/setup.bash

ros2 launch ur_setup_bringup \
  ur_real_moveit_robotiq_ur16e.launch.py \
  robot_ip:=192.168.56.101 \
  gripper_com_port:=/dev/robotiq
```

기본적으로 `scaled_joint_trajectory_controller`는 비활성 상태로 시작하여 launch 직후 자동 모션을 방지합니다.

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ur_setup_ws/install/setup.bash

ros2 control list_hardware_interfaces
ros2 control list_controllers
```

예상 controller 상태:

```text
joint_state_broadcaster                active
io_and_status_controller               active
speed_scaling_state_broadcaster        active
force_torque_sensor_broadcaster        active
scaled_joint_trajectory_controller     inactive
robotiq_activation_controller          active
robotiq_gripper_controller             active
```

Joint State 확인:

```bash
ros2 topic echo /joint_states --once
```

TF 확인:

```bash
ros2 topic echo /tf_static --once | head
```

UR 6축 joint와 Robotiq joint, `base_link`, `tool0`, `pedestal_link` 등이 확인되어야 합니다.

---

## 9. 안전 테스트 순서

### 9.1 그리퍼 단독 테스트

로봇팔을 정지시킨 상태에서 수행합니다.

그리퍼 닫기:

```bash
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.7, max_effort: 50.0}}" \
  --feedback
```

그리퍼 열기:

```bash
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 50.0}}" \
  --feedback
```

물리적으로 그리퍼가 움직이고 action 결과가 `SUCCEEDED`이면 정상입니다.

### 9.2 RViz Plan Only 검증

`scaled_joint_trajectory_controller`가 `inactive`인 상태에서 진행합니다.

1. RViz의 MotionPlanning 패널을 엽니다.
2. 목표 자세를 설정합니다.
3. `Plan`만 실행합니다.
4. `Plan & Execute`는 누르지 않습니다.
5. 궤적과 관절 제한을 확인합니다.
6. pedestal, plate, pallet 충돌 모델을 확인합니다.

### 9.3 실제 trajectory 실행 권한 활성화

계획 결과를 확인한 후 실행합니다.

```bash
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller
```

확인:

```bash
ros2 control list_controllers | grep scaled
```

최초 실행 시:

- Teach Pendant speed slider를 10~20%로 설정
- 짧고 안전한 경로부터 테스트
- 비상정지 버튼을 즉시 누를 수 있도록 준비

### 9.4 Pick & Place 노드 실행

터미널 3:

```bash
source /opt/ros/humble/setup.bash
source ~/ur_setup_ws/install/setup.bash

ros2 launch pick_place_module \
  pick_place.launch.py \
  use_sim:=false
```

확인:

```bash
ros2 action list | grep -E "pick|place|gripper_cmd"
ros2 node info /pick_place_node
```

안전한 빈 공간으로 소량 이동 테스트:

```bash
ros2 topic pub --once \
  /pick_goal \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.40, y: 0.20, z: 0.30},
      orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
    }}"
```

> 정확한 토픽 이름과 메시지 타입은 현재 `goal_relay_node` 구현을 확인하세요. 최초 테스트에서는 plate의 실제 작업 좌표가 아니라 장애물 위쪽의 안전한 빈 공간을 목표로 사용하세요.

안전 검증 후 실제 시퀀스를 수행합니다.

```bash
ros2 topic pub --once /pick_goal ...
ros2 topic pub --once /place_goal ...
```

---

## 10. 자주 발생하는 문제

| 증상 | 원인 및 해결 |
|---|---|
| `RTDE_ERROR`, `Could not connect` | UR IP, PC NIC, 방화벽, Ethernet 연결, External Control 프로그램 `Play` 상태 확인 |
| `Failed to load robotiq_driver/RobotiqGripperHardwareInterface` | `serial-ros2`, `robotiq_driver` 빌드 성공 여부와 `ros2 pkg prefix robotiq_driver` 확인 |
| `/dev/ttyUSB0` 또는 `/dev/robotiq` open 실패 | udev rule과 `dialout` 그룹 확인 후 재로그인 |
| MoveIt이 controller inactive로 실행 거절 | `scaled_joint_trajectory_controller` 활성화 명령 실행 |
| RViz에 pedestal/plate/pallet이 보이지 않음 | URDF/Xacro 위치 인수와 mesh 경로 확인 |
| `start_state_in_collision` | pedestal, plate, pallet의 실제 위치와 URDF 위치를 재측정하고 보정 |
| Polyscope 프로그램 중지 후 controller 비활성화 | External Control 프로그램을 다시 `Play`하고 trajectory controller 재활성화 |
| `libserial.a ... recompile with -fPIC` | `-DCMAKE_POSITION_INDEPENDENT_CODE=ON`으로 재빌드 |
| `ModuleNotFoundError: ros_gz_sim` | 실물 launch가 Gazebo 모듈을 import하는지 확인. 시뮬레이션 launch를 실행한 경우 Gazebo 설치 필요 |

---

## 11. 종료 및 재시작

종료:

1. 각 ROS 2 터미널에서 `Ctrl+C`
2. Teach Pendant에서 Program Stop
3. 필요 시 Robot Power OFF

Emergency Stop 또는 External Control 프로그램 중단 후에는 연결이 끊길 수 있습니다.

재시작 순서:

1. UR 상태 복구
2. Brake Release
3. External Control 프로그램 `Play`
4. ROS 2 launch 재실행
5. RViz에서 Plan Only 검증
6. trajectory controller 재활성화

```bash
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller
```