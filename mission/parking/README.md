# 주차 연결

P 표지를 보면 **가까운 왼쪽 절반 ROI**로 접근한다. 가로 범위는 공사 구간과 같고, 진입 동안만 위쪽을 더 잘라낸다. 흰 정지선을 만나면 기존 순서의 6단계를 **`/odom`의 이동 거리·회전각**으로 실행한다. 마지막 실제 정지를 확인하면 표지판을 기다리지 않고 **고정된 왼쪽 진행으로 인계**한다. driving은 주차 완료 시점부터 약5초 동안 왼쪽 ROI를 추종하고 일반 차선 추종으로 돌아간다.

주차 진입이 시작되면 `mission_model`의 표지판 인식을 잠시 중지하고 6단계 완료·좌회전 인계 시 재개한다. 인계는 표지판 입력과 무관하며, 표지판 검출 상태와 누적 횟수는 기존 중지·재개 처리에 따라 초기화된다. 차선 인식은 계속하며 표지 디버그 영상도 `signs paused: parking` 표시와 함께 갱신한다.

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

`config/parking_sim.yaml`의 `direction`으로 왼쪽·오른쪽 주차 중 하나를 선택한다. 각 쪽의 6단계 거리·회전각·선속도·각속도를 서로 독립적으로 수정할 수 있다. 현재 사용 중이던 YAML 튜닝값을 양쪽에 복사했으며, `direction: right` 선택도 유지했다.

```yaml
parking_detect_node:
  ros__parameters:
    direction: right  # left: 왼쪽 주차 / right: 오른쪽 주차

    # 인덱스:              0직선 1회전 2직선 3직선 4회전 5직선
    left.distances:      [0.55, 0.0,  0.32, 0.28, 0.0,  0.70]
    left.angles_deg:     [0.0,  89.4, 0.0,  0.0,  87.3, 0.0]
    left.linear_speeds:  [0.2,  0.0,  0.2, -0.2,  0.0,  0.2]
    left.angular_speeds: [0.0,  1.2,  0.0,  0.0,  1.5,  0.0]

    right.distances:      [0.55, 0.0,  0.32, 0.28, 0.0,  0.70]
    right.angles_deg:     [0.0,  89.4, 0.0,  0.0,  87.3, 0.0]
    right.linear_speeds:  [0.2,  0.0,  0.2, -0.2,  0.0,  0.2]
    right.angular_speeds: [0.0,  1.2,  0.0,  0.0,  1.5,  0.0]
```

| 항목 | 단위·지정 방법 |
| --- | --- |
| `<방향>.distances` | 거리[m], 양수. 회전 단계는0.0 |
| `<방향>.angles_deg` | 회전각[도], 양수. 직선 단계는0.0 |
| `<방향>.linear_speeds` | 최대 선속도[m/s], 양수=전진·음수=후진 |
| `<방향>.angular_speeds` | 최대 각속도[rad/s], 양수 크기. left는 좌회전·right는 우회전 |

배열은 각각 숫자6개이며 실수는 `90.0`처럼 소수점을 유지한다. 예를 들어 왼쪽 주차의 첫 이동 거리를30cm로 바꾸려면 `left.distances` 첫 값을 `0.30`으로 수정한다. 오른쪽 주차의 첫 회전을90도로 바꾸려면 `right.angles_deg` 두 번째 값을 `90.0`으로 수정한다. 첫 직선을 후진으로 바꾸려면 해당 쪽 `linear_speeds` 첫 값을 `-0.2`로 바꾼다. 거리에는 음수를 넣지 않는다. 좌·우 모두 두 회전은 선택한 direction 방향으로 진행한다.

`direction: left`이면 네 개의 `left.*` 배열, `right`이면 네 개의 `right.*` 배열을 읽는다. 반대쪽 배열은 선택한 주차의 동작값에 영향을 주지 않는다. 양쪽 배열 모두 시작 시6개 길이·유한값·직선/회전 구성을 검증하므로 잘못된 반대쪽 설정도 고친 뒤 실행해야 한다. 흰선 비율, 감속 게인, 정지 허용 오차, odom 수신 제한, 단계별 최대 시간 `step_timeouts`는 공통값이다.

기존 공통 `distances`·`angles_deg`·`linear_speeds`·`angular_speeds`는 `<방향>.항목`으로 옮겼다. 다른 launch나 별도 params 파일에서도 같은 이름으로 수정한다. 이 분리는 주차6단계 동작 설정에 적용한다. 접근은 기존 왼쪽 ROI 차선 추종을 사용한다. 6단계 완료 후에는 왼쪽·오른쪽 주차 설정 모두 고정 좌회전으로 인계한다. 마지막 직선 단계가 좌회전 시작 위치까지 로봇을 이동시키도록 필요하면 `left.distances`·`right.distances`의 여섯 번째 숫자를 각각 조절한다. `/parking/lane_mode`의 `left`는 기존 접근 모드 이름이며 `direction` 설정과 구분한다.

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

현재 YAML의 `white_ratio`는 사용자가 조정한 `0.03`이고 코드 기본값은 `0.04`다. 이전 YAML의 `0.60`은 기존 bringup에서 파일을 읽지 않아 적용되지 않던 값이다.

Galactic에서는 `rclpy.init(args=args)`를 사용하고 초기화 후 SIGINT·SIGTERM 핸들러를 등록한다. 지원하지 않는 `SignalHandlerOptions`와 `signal_handler_options` 인자를 사용하지 않으며, 진행 중 종료 시 ROS context를 닫기 전에 HALT와 정지 속도를 발행한다.

`onnx` 브랜치의 CUDA 기본값과 driving·mission_model의 `.venv` 실행 설정은 유지한다. parking은 추론 모델 없이 ROS·NumPy·OpenCV를 사용하며, 기존 런치에서 선택한 Python에 이 의존성이 필요하다. 패키지 빌드 후 설치본을 실행하는지, `.venv`에서 소스를 직접 실행하는지에 따라 변경 반영 방법이 다르므로 해당 런치의 실행 경로를 확인한다.

신호등 없이 수동 시험할 때는 같은 ROS 환경의 다른 터미널에서 주행을 허용한다.

```bash
ros2 topic pub --once /lane_state std_msgs/msg/Bool '{data: true}'
# 표지 없이 주차 접근 요청 또는 HALT에서 수동 재개 (주행 허용도 필요):
ros2 topic pub --once /parking_state std_msgs/msg/Bool '{data: true}'
# 진행 중 주차 일시 정지:
ros2 topic pub --once /parking_state std_msgs/msg/Bool '{data: false}'
# 외부 주행 정지: 진행 중 주차 동작도 HALT로 정지한다.
ros2 topic pub --once /lane_state std_msgs/msg/Bool '{data: false}'
ros2 topic echo /parking/status std_msgs/msg/String --qos-durability transient_local
```

완료 후에는 P 표지나 True 입력으로 주차를 재시작하지 않는다. HALT에서는 아래 조건에 따라 중단된 동작을 이어간다. 처음부터 다시 시험하려면 로봇을 정지·초기화하고 주행·표지·주차 노드를 재시작한다. 별도의 주차 초기화 서비스는 없다.

HALT 동안은 20Hz로 0속도를 유지하고, 단계 번호와 시작 위치·누적 yaw 원점을 보존한다. 같은 odom 좌표계에서 재개하면 정지 중 잔여 이동을 포함하여 원래 목표까지 남은 거리·각도만 수행한다. `PARKING` 복귀 시 `maneuver`, `APPROACH` 복귀 시 `left` 모드를 다시 발행한다.

- 일시적인 odom 누락·지연·잘못된 값, 잘못된 접근 영상은 정상 입력이 복구되면 자동 재개한다.
- 외부 `/lane_state=False`, 수동 `/parking_state=False`, 단계·정지 확인 timeout은 자동 재개하지 않는다. 원인을 해결하고 `/lane_state=True`로 주행을 허용한 뒤 `/parking_state=True`로 재개를 요청한다. P 표지와 `/lane_state=True`만으로는 재개되지 않는다. HALT 중 외부 정지·수동 정지를 받으면 대기 중이던 자동 재개도 해제한다.
- 재개에는 HALT 정지 명령 이후의 서로 다른 stamp를 가진 최신 odom 3개가 연속으로 필요하며, 모두 정지 속도 허용 오차 이하여야 한다. 접근 복귀는 정상 영상도 연속 3개 필요하다. 재개 확인 odom과 영상은 `odom_timeout` 이내의 최신 입력이어야 한다. 재개 요청을 먼저 보내도 이 조건을 만족할 때까지 0속도를 유지한다.
- 정지 시간은 단계 timeout에서 제외한다. 단계 timeout 뒤 명시적으로 재시도하면 남은 목표를 유지하면서 해당 단계의 시간 예산을 새로 부여한다. 정지 확인 중이었다면 재개 명령 이후의 새 odom으로 다시 정지를 확인한 뒤 다음 단계로 넘어간다.
- 주차 중 또는 HALT 중 odom stamp 역행·frame 변경, 진행 중 또는 HALT 중 노드 시간 역행, 노드 종료는 재개를 차단한다. 원래 목표의 기준을 신뢰할 수 없으므로 위치·시간 설정을 확인하고 노드를 재시작해야 한다.

## 동작 순서

1. `IDLE`: `/parking=True`를 기억하고 `/lane_state=True`까지 기다린다. 표지가 사라져도 요청은 유지한다.
2. `APPROACH`: 왼쪽 절반 ROI의 상단을250행으로 내려 x[76,326), y[250,480)에서 차선을 추종한다. 미검출이면 설정한 고정 좌회전 탐색, 재검출이면 기존 추종을 한다. 방향 표지판의 위쪽 확장은 사용하지 않는다. 이때부터 표지판 YOLO 추론을 중지하고 기존 True 표지 상태를 False로 해제한다.
3. `PARKING`: `/bev`의 왼쪽 230열 전체 높이에서 HSV [0,0,160]~[180,40,255]의 흰 비율이 `white_ratio`(현재 코드 기본0.04)를 초과하면 차선 속도 출력을 막는다. 한 프레임의 조건으로 시작하며, 실제 정지를 확인한 뒤 아래 6단계를 실행한다. 빈 ROI는 오류로 처리하며 흰 픽셀이 없는 정상 영상의 비율은 0이다.
4. `DONE`: 마지막 목표 도달 후 **정지 명령 이후의 새 odom으로 실제 정지**를 확인하면 `step=6`과 함께 `/parking/lane_mode=intersection`을 한 번 발행한다. `EXIT` 대기 상태나 `/left` 구독 없이 기존 교차로·좌회전 ROI로 바로 인계한다. 표지 인식도 이때 재개한다. driving은 왼쪽 ROI를 `parking.exit_follow_sec`(기본5초) 동안 유지한 뒤 ROI와 주차 출구 교차로 상태를 함께 해제한다. 이5초 동안 좌·우 표지나 같은 인계 메시지를 다시 받아도 방향·시작 시각을 바꾸지 않는다.

| 단계 | 현재 YAML의 동작 | 거리(m) | 회전각(도) | 최대 선속도(m/s) | 최대 각속도 크기(rad/s) |
|---|---|---:|---:|---:|---:|
| 0 | 전진 | 0.55 | 0 | 0.2 | 0 |
| 1 | 선택 방향 회전1 | 0 | 89.4 | 0 | 1.2 |
| 2 | 전진 | 0.32 | 0 | 0.2 | 0 |
| 3 | 후진 | 0.28 | 0 | -0.2 | 0 |
| 4 | 선택 방향 회전2 | 0 | 87.3 | 0 | 1.5 |
| 5 | 전진 | 0.70 | 0 | 0.2 | 0 |

표는 좌·우 분리 시 복사한 기존 YAML 튜닝값이다. 이후 각 방향의 배열을 따로 바꿀 수 있다. `direction: left`이면 두 회전 모두 왼쪽, `right`이면 모두 오른쪽이다. 20Hz 타이머가 odom을 확인한다. 직선은 시작 방향으로 투영한 부호 있는 변위, 회전은 ±180° 경계를 보정한 누적 yaw 변화로 진행량을 계산한다. 옆으로 움직이거나 반대 방향으로 움직인 거리를 완료로 계산하지 않는다.

남은 거리·각도에 비례해 감속하며 설정한 최소·최대 속도로 제한한다. 직선 구간은 `heading_kp`로 시작 방향을 유지하고 각속도를 `max_heading_angular`로 제한한다. 남은 거리가 `distance_tolerance`(기본1cm), 남은 각도가 `angle_tolerance_deg`(기본1도) 이하이거나 목표를 통과하면 정지 명령을 보낸다. 이후 **정지 명령 이후 시각의 새 odom**에서 실제 속도가 `stop_linear_tolerance`/`stop_angular_tolerance` 이하임을 확인하고 다음 단계의 기준 위치·방향을 잡는다. 마지막 단계도 정지를 확인한 뒤 표지판 대기 없이 DONE·고정 좌회전으로 인계한다. 정지 중의 잔여 이동을 다음 단계 거리로 계산하지 않는다.

`step_timeouts`는 단계별 최대 시간이며 `stop_timeout`은 정지 대기 최대 시간이다. 초과하면 HALT로 정지하고 다음 단계로 넘기지 않는다. odom이 없거나, 수신 시각이 `odom_timeout`보다 오래됐거나, 메시지 stamp와 노드 시각의 차이가 `odom_timeout`을 넘거나, 유효하지 않거나, 주차 동작 중 stamp가 역행하거나 frame이 바뀌면 HALT로 정지한다. 같은 stamp의 반복 메시지로 수신 시각이나 정지 확인을 갱신하지 않는다. 목표 거리·각도는 허용 오차보다 커야 하고, 최대 속도는 해당 최소 속도 이상이어야 하며 잘못된 설정은 노드 시작 시 거부한다.

설정은 `share/parking/config`에도 설치된다. bringup에서는 YAML 값이 우선하고 파일에 없는 값은 코드 기본값을 쓴다. YAML을 지정하지 않으면 기존 시간 설정에서 환산했던 코드 기본 배열을 쓰므로, 현재 튜닝한 YAML과 동작값이 다를 수 있다. ROI의 기본 너비는 driving의 `yolo_lane.roi_half_width`, 출차 상단은 `yolo_lane.roi_top_row`를 따른다. odom은 추정 위치이므로 바퀴 미끄러짐에 의한 실제 거리·각도 오차를 없애지는 못한다. 목표 통과 후에는 다시 반대 방향으로 보정하지 않으며 제동 중 실제 정지 위치는 목표보다 더 진행할 수 있다.

진입 전용 값은 `/driving`의 아래 파라미터로 조절한다. 실행 중 설정은 다음 처리 프레임부터 적용되며 재시작하면 코드 기본값으로 돌아간다. 주차 YAML이 아니라 driving 파라미터다.

```bash
ros2 param set /driving parking.approach_roi_top_row 250
ros2 param set /driving pure_pursuit.v_lost 0.05
```

상단 행을 높이면 더 가까운 영역만 본다. `pure_pursuit.v_lost`는 주차를 포함한 YOLO 정상 미검출의 직진 속도다. `parking.search_speed`와 `parking.search_angular`는 사용자가 활성화한 진입 미검출 탐색에 적용하며 driving 노드에서 조정한다. 실제 진입 성공에 맞춰 보정한 값은 아니다.

## 제어 연결

주차는 다른 미션의 속도를 중계하지 않는다. driving과 미션들이 기존 `/cmd_vel`을 직접 사용하고, 주차는 고정동작과 정지·취소 때만 속도를 발행한다. HALT 동안은 주기적으로 0속도를 유지하며 대기·접근·출차·완료 동안에는 보내지 않는다. 다른 직접 제어 미션과 순서대로 동작하는 구조이며 동시 실행 우선순위를 중재하는 기능은 없다.

- `/lane_state` Bool: 외부 주행 허용. 주차도 구독하지만 이 토픽에 자기 정지 신호를 재발행하지 않는다.
- `/parking` Bool: mission_model의 P표지 상태. 기존 연속 검출 확인을 재사용한다. parking은 `/left`를 구독하지 않는다.
- `/parking_state` Bool: True는 수동 시작 요청 또는 HALT 재개 요청, False는 일시 정지 입력. 자기재발행하지 않는다.
- `/odom` Odometry: 위치·자세·실제 속도 입력. 주차 6단계를 실행하는 동안 최신 메시지가 필요하다.
- `/parking/status` String: 상태 변경, 각 단계 시작, 목표 도달 시 `phase`, `step`, `reason`, `direction`을 담은 JSON을 발행한다. `step`은0~5이며6은 모든 동작 완료다.
- `/parking/lane_mode` String: `left`는 가까운 왼쪽ROI와 미검출 시 고정 좌회전 탐색, `maneuver`는 검출하면서 차선 속도 출력만 중지, `halt`는 중지 유지, `intersection`은 6단계 완료 후 주차 제한 해제·5초 좌회전 인계, 빈 문자열은 일반 상태다.

`parking.exit_follow_sec`는 **driving 노드**의 시작 전용 읽기 전용 파라미터이며 유한한 양수[초]다. 주차 스텝 YAML의 배열과는 별개다. 기본5초는 driving이 `intersection` 인계를 받은 ROS 시각부터 센다. 고정 각속도나 odom 기준90도 회전이 아니라 기존 왼쪽 ROI·Pure Pursuit 추종이며, 교차로 속도 정책을 사용한다. 명시적 `/intersection_state=false`나 새 주차 모드는5초가 끝나기 전에도 출구 추종을 해제한다. 일반 표지판의 `sign.roi_hold_sec`(기본30초)와 외부 교차로 상태 유지 방식은 기존대로다.

`mission_model`도 `/parking/lane_mode`를 transient local로 구독한다. `left`/`maneuver`/`halt`에서는 표지판 추론·검출 누적을 쉬고, `exit`/`intersection`/빈 문자열에서는 재개한다. 중지·재개 전환 때 이전 영상·카운터·활성 표지를 초기화하므로 계속 화면에 남아 있는 좌회전 표지도 출차 후 다시 True를 발행할 수 있다. P표지를 보기 전 일반 주행의 표지 처리는 유지된다.

외부 `/lane_state=True`는 `maneuver`/`halt`의 차선 출력을 강제로 켜지 않는다. 외부 False, 수동 취소, 잘못된 입력 영상, 진행 중 시뮬레이션 시간 역행은 HALT로 정지한다. 진행 중 노드를 정상 종료할 때도 정지한다. 종료 뒤 ROS publish나 `/parking_state` 자기구독 루프는 없다.

6단계 후 좌회전 인계는 `/left` 수신과 무관하게 한 모드 메시지로 처리한다. 주차 완료 전 외부 정지·취소·odom 오류가 발생하면 HALT 동안 좌회전으로 인계하지 않는다. 재개 후 남은 목표와 마지막 정지를 확인해야 인계한다. 이후 방향 표지와 교차로 상태 메시지는 기존 방식대로 처리된다. 별도의 속도 중계 remap은 필요 없다. 주차의 상태·모드·진입 시 정지·로그는 `set_phase()` 한 곳에서 함께 갱신한다.

## 확인 범위

관련 검사는 [주차 제어](test/test_parking_control.py), [driving 연결](../../driving/test/test_parking_handoff.py), [표지 인식 중지](../mission_model/test/test_parking_sign_pause.py)에 있다.

HALT 재개 검사는 좌·우 6단계 각각의 남은 목표 보존, 정지 중 잔여 이동, 긴 중단 시간의 timeout 제외, 최신 정지 odom·영상의 연속 확인, 수동 재개와 외부 정지 우선, HALT 중 좌표·시간 초기화 시 복귀 차단, 초기·마지막 정지 확인 중 재개를 확인한다. ROS publisher는 메모리 기록기로 대체하며 실제 로봇에는 명령을 보내지 않는다.

ROS context와 모델을 대체한 검사로 흰선 ROI와 6단계 거리·각도·종료, 후진 부호·횡방향 이동 제외, ±180° 경계 회전, 감속·방향 보정, 새 odom을 통한 정지 확인, odom 누락·지연·중복·미래 stamp·frame 변경·시간 역행, 시간 초과, 외부 정지·취소, 출차 인계를 확인한다. 좌·우에 서로 다른 거리·각도·속도를 지정한6단계를 각각 실행하고 선택한 쪽의 값만 사용하는지 확인한다. 각 직선 단계는 선속도 부호에 따라 전진·후진하는지 확인한다. 가감속을 포함한 단순 운동 모델에서 첫 단계를 전진 또는 후진으로 설정하고 좌·우 주차 모두 표지판 없이 DONE·고정 좌회전 인계로 끝나는지 확인한다. driving의 BEV 영상 발행은 인계 검사에서 별도 mock으로 대체한다.

주차 완료 자동 좌회전·주차 전용5초 유지 변경 후 Galactic/Python 3.8에서 위 세 테스트 파일의 **123개 검사**와 parking 패키지 symlink 빌드를 통과했다. 설치된 YAML을 읽어 오른쪽은 기존 튜닝값, 왼쪽은 검증용 파일에서 거리·각도·선속도·각속도를 별도로 바꾼 값으로 각각 DDS6단계·출차 인계·odom 끊김 정지를 확인했다. 원본 YAML의 좌우 배열 초기값은 분리 전 튜닝값과 동일하게 유지했다. 별도 ROS 도메인·검증용 토픽에서 설치본의 YAML 적용과 읽기 전용 파라미터 거부를 확인하고, 합성 영상·odom을 DDS로 보내 IDLE → APPROACH → PARKING의 6단계 → DONE·고정 좌회전 인계를 확인했다. 정지 전 속도가 남아 있으면 단계를 넘기지 않고 odom을 끊으면 HALT와 최종0속도를 발행하는 것도 확인했다. 실제 로봇의 `/cmd_vel`에는 명령을 보내지 않았다. 설치된 parking·driving을 별도 ROS 도메인에서 함께 실행하여 좌·우 주차 모두 표지 입력 없는 좌회전 인계, 약5초 후 일반 추종 복귀, 중간 표지의 시간 연장 방지, 일반 표지30초 정책 유지를 DDS로 확인했다. 실제 로봇의 속도 토픽에는 연결하지 않았다. SIGINT·SIGTERM의 정지·정상 종료는 이전 시간 기반 버전에서 확인한 이력이며 이번 변경에서는 재검증하지 않았다.

기존 사용자 시험 로그에서는 P 검출 후 APPROACH와 표지 중지를 확인했지만, 이번 자동 좌회전·5초 유지 동작의 실차 기록은 없다. 실제 흰선 ROI·비율과 odom 오차를 계측하고 6동작·출차·교차로 인계를 검증하는 작업이 남아 있다. 이번 검사는 실제 카메라·CUDA 추론·실차 주차 성공을 보장하지 않는다.
