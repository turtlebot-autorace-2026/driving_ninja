# 주차 연결

P 표지를 보면 **가까운 왼쪽 절반 ROI**로 접근한다. 가로 범위는 공사 구간과 같고, 진입 동안만 위쪽을 더 잘라낸다. 흰 정지선을 만나면 기존 순서의 6단계를 **`/odom`의 이동 거리·회전각**으로 실행하고, 차선 주행을 재개한다. 출차 중 주차장 옆 좌회전 표지를 확인하면 기존 교차로·좌회전 처리로 넘긴다.

주차 진입이 시작되면 `mission_model`의 표지판 인식을 잠시 중지하고 6단계가 끝나 출차할 때 재개한다. 진입 전에 본 표지와 누적 검출 횟수를 비우므로, 출차 후 새 영상에서 다시 연속 확인한 좌회전 표지만 인계에 사용한다. 차선 인식은 계속하며 표지 디버그 영상도 `signs paused: parking` 표시와 함께 갱신한다.

접근·출차 조향은 `driving`이 담당한다. 사용자가 주석을 해제한 진입 미검출 탐색은 `parking.search_speed`·`parking.search_angular`(현재 코드 기본0.04m/s·0.55rad/s)로 전진·좌회전한다. 출차·교차로 미검출은 `pure_pursuit.v_lost`(코드 기본0.05m/s)로 직진하고 각속도는0이다. 차선을 다시 찾은 프레임부터 기존 YOLO/Pure Pursuit 추종으로 돌아간다. 실제 값은 사용하는 launch의 설정에 따라 달라질 수 있다.

odom 제어는 `PARKING`의 6단계에 적용한다. 직선 구간은 단계 시작 방향을 유지하고, 회전 구간은 단계 시작 이후의 yaw 변화를 누적한다. 접근·출차는 기존 차선 추종이며, YOLO 모델과 `lane_estimator.py`/`pure_pursuit.py` 계산은 그대로다. `detection (copy).py`는 실행하지 않는다.

## 기존 런치에 연결

팀에서 사용하는 런치에서 아래 세 노드를 함께 실행한다. 이 변경에는 별도의 통합 런치나 시뮬레이션 자산을 포함하지 않는다.

| 패키지 / 실행 항목 | 역할 |
| --- | --- |
| `driving` / `driving` | 차선 추종, 주차 ROI와 속도 출력 제한 |
| `mission_model` / `detection` | P·좌회전 표지 검출, 주차 중 표지 추론 중지 |
| `parking` / `detection` | 흰선 판정, odom 기반 6동작, 출차 인계 |

parking을 포함해야 `/parking/lane_mode`가 발행되어 진입 ROI와 표지 중지가 적용된다. 세 노드는 같은 ROS 도메인에서 연결하고, 시뮬레이션에서는 odom 발행 노드까지 모두 같은 `use_sim_time` 설정을 사용한다. driving은 `/camera`, mission_model은 `/camera2`, parking의 흰선 검출은 driving이 만든 `/bev`를 사용한다. 영상 타입은 `sensor_msgs/msg/CompressedImage`다. `/bev`가 발행되려면 driving의 `yolo_lane.publish_debug=true`가 필요하다.

로컬 `/home/atom5/turtlebot3_ws/bringup.launch.py`에는 parking과 mission_model 노드가 연결되어 있다. parking은 설치된 `share/parking/config/parking_sim.yaml`을 `parameters`로 읽는다. odom용 새 노드는 추가하지 않는다. 기존 로봇/시뮬레이터가 `nav_msgs/msg/Odometry`를 `/odom`으로 발행해야 한다. 다른 이름이면 parking 노드에 `remappings=[('/odom', '/실제_토픽')]`을 지정한다. 이미 실행 중인 런치는 재시작해야 반영된다.

## 숫자 조정

`config/parking_sim.yaml`에서 같은 배열 위치가 같은 단계다. 아래 네 줄의 숫자를 바꾸면 거리·각도·속도를 조정할 수 있다. 배열은 각각 6개이며, 실수는 `90.0`처럼 소수점을 유지한다.

```yaml
# 단계:           0후진  1회전1  2전진  3후진  4회전2  5전진
distances:       [0.20,  0.0,    0.32,  0.32,  0.0,    0.20]  # m, 양수
angles_deg:      [0.0,   89.4,   0.0,   0.0,   77.3,   0.0]   # 도, 양수
linear_speeds:   [-0.2,  0.0,    0.2,  -0.2,   0.0,    0.2]   # m/s, 최대값
angular_speeds:  [0.0,   1.2,    0.0,   0.0,   1.5,    0.0]   # rad/s, 최대값
```

예를 들어 첫 후진만 30cm로 늘리려면 `distances` 첫 숫자를 `0.30`으로 바꾼다. 첫 회전을 90도로 바꾸려면 `angles_deg` 두 번째 숫자를 `90.0`으로 바꾼다. 회전 단계의 거리는 `0.0`, 직선 단계의 각도는 `0.0`으로 유지한다. 각 직선 단계의 방향은 `linear_speeds`의 부호로 정한다(양수=전진, 음수=후진). **첫 단계를 전진으로 바꾸려면 `linear_speeds` 첫 값을 `-0.2`에서 `0.2`로 바꾸고 `distances` 첫 값은 양수 `0.20`을 유지한다.** 거리에는 음수를 쓰지 않는다. `direction: left/right`는 두 회전의 방향을 정한다.

```bash
cd /home/atom5/turtlebot3_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select parking --symlink-install
source install/setup.bash
ros2 launch ./bringup.launch.py
```

현재 설치는 symlink 방식이므로 빌드 이후 YAML 숫자만 고칠 때는 런치를 재시작하면 된다. 일반 복사 설치에서는 파일 수정 후 다시 빌드한다. 모든 설정은 시작 시 읽기 전용이며 실행 중 `ros2 param set`으로 바꾸지 않는다. parking만 따로 실행할 때는 다음처럼 같은 파일을 지정한다.

```bash
ros2 run parking detection --ros-args --params-file \
  /home/atom5/turtlebot3_ws/src/package_n/mission/parking/config/parking_sim.yaml
```

현재 YAML과 코드의 `white_ratio`는 사용자가 조정한 `0.04`다. 이전 YAML의 `0.60`은 기존 bringup에서 파일을 읽지 않아 적용되지 않던 값이다.

Galactic에서는 `rclpy.init(args=args)`를 사용하고 초기화 후 SIGINT·SIGTERM 핸들러를 등록한다. 지원하지 않는 `SignalHandlerOptions`와 `signal_handler_options` 인자를 사용하지 않으며, 진행 중 종료 시 ROS context를 닫기 전에 HALT와 정지 속도를 발행한다.

`onnx` 브랜치의 CUDA 기본값과 driving·mission_model의 `.venv` 실행 설정은 유지한다. parking은 추론 모델 없이 ROS·NumPy·OpenCV를 사용하며, 기존 런치에서 선택한 Python에 이 의존성이 필요하다. 패키지 빌드 후 설치본을 실행하는지, `.venv`에서 소스를 직접 실행하는지에 따라 변경 반영 방법이 다르므로 해당 런치의 실행 경로를 확인한다.

신호등 없이 수동 시험할 때는 같은 ROS 환경의 다른 터미널에서 주행을 허용한다.

```bash
ros2 topic pub --once /lane_state std_msgs/msg/Bool '{data: true}'
# 표지 없이 수동으로 주차 접근을 요청할 때 (주행 허용도 필요):
ros2 topic pub --once /parking_state std_msgs/msg/Bool '{data: true}'
# 진행 중 주차 취소:
ros2 topic pub --once /parking_state std_msgs/msg/Bool '{data: false}'
# 외부 주행 정지: 진행 중 주차 동작도 HALT로 취소된다.
ros2 topic pub --once /lane_state std_msgs/msg/Bool '{data: false}'
ros2 topic echo /parking/status std_msgs/msg/String --qos-durability transient_local
```

완료·취소 후에는 P 표지나 True 입력으로 주차를 재시작하지 않는다. 다시 시험하려면 로봇을 정지·초기화하고 주행·표지·주차 노드를 재시작한다. 별도의 주차 초기화 서비스는 없다.

## 동작 순서

1. `IDLE`: `/parking=True`를 기억하고 `/lane_state=True`까지 기다린다. 표지가 사라져도 요청은 유지한다.
2. `APPROACH`: 왼쪽 절반 ROI의 상단을250행으로 내려 x[76,326), y[250,480)에서 차선을 추종한다. 미검출이면 설정한 고정 좌회전 탐색, 재검출이면 기존 추종을 한다. 방향 표지판의 위쪽 확장은 사용하지 않는다. 이때부터 표지판 YOLO 추론을 중지하고 기존 True 표지 상태를 False로 해제한다.
3. `PARKING`: `/bev`의 왼쪽 320열 전체 높이에서 HSV [0,0,160]~[180,40,255]의 흰 비율이 `white_ratio`(현재 코드 기본0.04)를 초과하면 차선 속도 출력을 막는다. 한 프레임의 조건으로 시작하며, 실제 정지를 확인한 뒤 아래 6단계를 실행한다.
4. `EXIT`: 마지막 동작 후 실제 정지를 확인하면 차선 주행을 재개하고 기존 왼쪽 절반 ROI(x[76,326),y[150,480))로 나간다. 미검출 시 저속 직진을 유지한다. 표지 인식도 재개하며 대기 영상과 검출 횟수를 비우고 새로 받은 영상부터 기존 `confirm_frames`(코드 기본2) 연속 확인을 시작한다.
5. `DONE`: 출차 후 새 `/left=True`가 오면 주차 ROI를 풀고 기존 교차로 모드와 좌회전 ROI를 적용한다. 새 노드를 띄우지 않는다. IDLE/APPROACH/PARKING 중 받은 좌회전 표지는 저장하지 않아 EXIT 진입만으로 즉시 인계하지 않는다.

| 단계 | 동작 | 거리(m) | 회전각(도) | 최대 선속도(m/s) | 최대 회전 속도(rad/s) |
|---|---|---:|---:|---:|---:|
| 0 | 후진 | 0.20 | 0 | -0.2 | 0 |
| 1 | 회전1 | 0 | 89.4 | 0 | 1.2 |
| 2 | 전진 | 0.32 | 0 | 0.2 | 0 |
| 3 | 후진 | 0.32 | 0 | -0.2 | 0 |
| 4 | 회전2 | 0 | 77.3 | 0 | 1.5 |
| 5 | 전진 | 0.20 | 0 | 0.2 | 0 |

목표값은 기존 명령 속도×시간에서 환산한 초기값이며, 실차에서 측정해 보정한 값은 아니다. `direction: left`이면 두 회전 모두 왼쪽이다. 20Hz 타이머가 odom을 확인한다. 직선은 시작 방향으로 투영한 부호 있는 변위, 회전은 ±180° 경계를 보정한 누적 yaw 변화로 진행량을 계산한다. 옆으로 움직이거나 반대 방향으로 움직인 거리를 완료로 계산하지 않는다.

남은 거리·각도에 비례해 감속하며 설정한 최소·최대 속도로 제한한다. 직선 구간은 `heading_kp`로 시작 방향을 유지하고 각속도를 `max_heading_angular`로 제한한다. 남은 거리가 `distance_tolerance`(기본1cm), 남은 각도가 `angle_tolerance_deg`(기본1도) 이하이거나 목표를 통과하면 정지 명령을 보낸다. 이후 **정지 명령 이후 시각의 새 odom**에서 실제 속도가 `stop_linear_tolerance`/`stop_angular_tolerance` 이하임을 확인하고 다음 단계의 기준 위치·방향을 잡는다. 마지막 단계도 정지를 확인한 뒤 EXIT로 인계한다. 정지 중의 잔여 이동을 다음 단계 거리로 계산하지 않는다.

`step_timeouts`는 단계별 최대 시간이며 `stop_timeout`은 정지 대기 최대 시간이다. 초과하면 HALT로 정지하고 다음 단계로 넘기지 않는다. odom이 없거나, 수신 시각이 `odom_timeout`보다 오래됐거나, 메시지 stamp와 노드 시각의 차이가 `odom_timeout`을 넘거나, 유효하지 않거나, 주차 동작 중 stamp가 역행하거나 frame이 바뀌면 HALT로 정지한다. 같은 stamp의 반복 메시지로 수신 시각이나 정지 확인을 갱신하지 않는다. 목표 거리·각도는 허용 오차보다 커야 하고, 최대 속도는 해당 최소 속도 이상이어야 하며 잘못된 설정은 노드 시작 시 거부한다.

설정은 `share/parking/config`에도 설치되며 파일을 지정하지 않으면 같은 코드 기본값을 사용한다. ROI의 기본 너비는 driving의 `yolo_lane.roi_half_width`, 출차 상단은 `yolo_lane.roi_top_row`를 따른다. odom은 추정 위치이므로 바퀴 미끄러짐에 의한 실제 거리·각도 오차를 없애지는 못한다. 목표 통과 후에는 다시 반대 방향으로 보정하지 않으며 제동 중 실제 정지 위치는 목표보다 더 진행할 수 있다.

진입 전용 값은 `/driving`의 아래 파라미터로 조절한다. 실행 중 설정은 다음 처리 프레임부터 적용되며 재시작하면 코드 기본값으로 돌아간다. 주차 YAML이 아니라 driving 파라미터다.

```bash
ros2 param set /driving parking.approach_roi_top_row 250
ros2 param set /driving pure_pursuit.v_lost 0.05
```

상단 행을 높이면 더 가까운 영역만 본다. `pure_pursuit.v_lost`는 주차를 포함한 YOLO 정상 미검출의 직진 속도다. `parking.search_speed`와 `parking.search_angular`는 사용자가 활성화한 진입 미검출 탐색에 적용하며 driving 노드에서 조정한다. 실제 진입 성공에 맞춰 보정한 값은 아니다.

## 제어 연결

주차는 다른 미션의 속도를 중계하지 않는다. driving과 미션들이 기존 `/cmd_vel`을 직접 사용하고, 주차는 고정동작과 정지·취소 때만 속도를 발행한다. 대기·접근·출차·완료 동안 주기적으로0속도를 보내지 않는다. 다른 직접 제어 미션과 순서대로 동작하는 구조이며 동시 실행 우선순위를 중재하는 기능은 없다.

- `/lane_state` Bool: 외부 주행 허용. 주차도 구독하지만 이 토픽에 자기 정지 신호를 재발행하지 않는다.
- `/parking`, `/left` Bool: mission_model의 표지 상태. 기존 연속 검출 확인을 재사용한다.
- `/parking_state` Bool: 수동 시작 요청·취소 입력. 자기재발행하지 않는다.
- `/odom` Odometry: 위치·자세·실제 속도 입력. 주차 6단계를 실행하는 동안 최신 메시지가 필요하다.
- `/parking/status` String: 상태 변경, 각 단계 시작, 목표 도달 시 `phase`, `step`, `reason`을 담은 JSON을 발행한다. `step`은0~5이며6은 모든 동작 완료다.
- `/parking/lane_mode` String: `left`는 가까운 왼쪽ROI와 미검출 시 고정 좌회전 탐색, `exit`는 기존 왼쪽 절반ROI와 차선 추종, `maneuver`는 검출하면서 차선 속도 출력만 중지, `halt`는 중지 유지, `intersection`은 주차 제한 해제 및 기존 교차로·좌회전 처리, 빈 문자열은 일반 상태다.

`mission_model`도 `/parking/lane_mode`를 transient local로 구독한다. `left`/`maneuver`/`halt`에서는 표지판 추론·검출 누적을 쉬고, `exit`/`intersection`/빈 문자열에서는 재개한다. 중지·재개 전환 때 이전 영상·카운터·활성 표지를 초기화하므로 계속 화면에 남아 있는 좌회전 표지도 출차 후 다시 True를 발행할 수 있다. P표지를 보기 전 일반 주행의 표지 처리는 유지된다.

외부 `/lane_state=True`는 `maneuver`/`halt`의 차선 출력을 강제로 켜지 않는다. 외부 False, 수동 취소, 잘못된 입력 영상, 진행 중 시뮬레이션 시간 역행은 HALT로 정지한다. 진행 중 노드를 정상 종료할 때도 정지한다. 종료 뒤 ROS publish나 `/parking_state` 자기구독 루프는 없다.

출구의 좌회전 인계는 한 모드 메시지로 처리해 `/left`가 driving에 먼저 도착해도 놓치지 않는다. 인계 후에는 driving의 `config/intersection.yaml`에 있는 `intersection.left.*` 네 설정을 사용한다. `parking_sim.yaml`의 `direction`과 속도·각도 배열은 주차 6단계용이며 교차로 좌회전 설정과는 별개다. 교차로 종료 신호 또는 표지 ROI 유지 시간 종료 시에는 일반 주행 설정으로 돌아간다. 이후 방향 표지와 교차로 상태 메시지는 기존 방식대로 처리된다. 별도의 속도 중계 remap은 필요 없다. 주차의 상태·모드·진입 시 정지·로그는 `set_phase()` 한 곳에서 함께 갱신한다.

## 확인 범위

관련 검사는 [주차 제어](test/test_parking_control.py), [driving 연결](../../driving/test/test_parking_handoff.py), [표지 인식 중지](../mission_model/test/test_parking_sign_pause.py)에 있다.

ROS context와 모델을 대체한 검사로 흰선 ROI와 6단계 거리·각도·종료, 후진 부호·횡방향 이동 제외, ±180° 경계 회전, 감속·방향 보정, 새 odom을 통한 정지 확인, odom 누락·지연·중복·미래 stamp·frame 변경·시간 역행, 시간 초과, 외부 정지·취소, 출차 인계를 확인한다. 각 직선 단계는 선속도 부호에 따라 전진·후진하는지 확인한다. 가감속을 포함한 단순 운동 모델에서 첫 단계를 전진 또는 후진으로 설정하고 좌·우 주차 모두 EXIT로 끝나는지 확인한다. driving의 BEV 영상 발행은 인계 검사에서 별도 mock으로 대체한다.

이번 odom 변경은 Galactic/Python 3.8에서 위 세 테스트 파일의 **88개 검사**와 parking 패키지 symlink 빌드를 통과했다. 별도 ROS 도메인·검증용 토픽에서 설치본의 YAML 적용과 읽기 전용 파라미터 거부를 확인하고, 합성 영상·odom을 DDS로 보내 IDLE → APPROACH → PARKING의 6단계 → EXIT → DONE 인계를 확인했다. 정지 전 속도가 남아 있으면 단계를 넘기지 않고 odom을 끊으면 HALT와 최종0속도를 발행하는 것도 확인했다. 실제 로봇의 `/cmd_vel`에는 명령을 보내지 않았다. SIGINT·SIGTERM의 정지·정상 종료는 이전 시간 기반 버전에서 확인한 이력이며 이번 변경에서는 재검증하지 않았다.

기존 사용자 시험 로그에서는 P 검출 후 APPROACH와 표지 중지를 확인했지만, 실차에서 흰선 조건을 통과한 PARKING 및 EXIT 기록은 없었다. 실제 흰선 ROI·비율과 odom 오차를 계측하고 6동작·출차·교차로 인계를 검증하는 작업이 남아 있다. 이번 검사는 실제 카메라·CUDA 추론·실차 주차 성공을 보장하지 않는다.
