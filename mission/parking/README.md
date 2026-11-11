# 주차 연결

P 표지를 보면 **가까운 왼쪽 절반 ROI**로 접근한다. 가로 범위는 공사 구간과 같고, 진입 동안만 위쪽을 더 잘라낸다. 기존 흰 정지선을 만나면 원본 하드코딩 6단계를 실행하고, 차선 주행을 재개한다. 출차 중 주차장 옆 좌회전 표지를 확인하면 기존 교차로·좌회전 처리로 넘긴다.

주차 진입이 시작되면 `mission_model`의 표지판 인식을 잠시 중지하고 6단계가 끝나 출차할 때 재개한다. 진입 전에 본 표지와 누적 검출 횟수를 비우므로, 출차 후 새 영상에서 다시 연속 확인한 좌회전 표지만 인계에 사용한다. 차선 인식은 계속하며 표지 디버그 영상도 `signs paused: parking` 표시와 함께 갱신한다.

접근·출차 조향은 `driving`이 담당한다. 진입 중 유효 차선을 놓치면 `0.05m/s` 전진 + 왼쪽 `0.8rad/s` 조향으로 탐색한다. 차선을 다시 찾은 프레임부터 기존 YOLO/Pure Pursuit 추종으로 돌아간다. 출차에는 탐색 조향을 적용하지 않으며 기존 미검출 정책인 `pure_pursuit.v_lost`(코드 기본값 `0.05m/s` 직진)를 따른다. 실제 값은 사용하는 launch의 설정에 따라 달라질 수 있다.

시간·odom으로 끝내는 고정 진입 원호, 차선 접선각 판정, odom 방향 유지·출구각, 양쪽 차선 복귀 판정은 사용하지 않는다. YOLO 모델과 `lane_estimator.py`/`pure_pursuit.py` 계산은 그대로다. `detection (copy).py`는 실행하지 않는다.

## 기존 런치에 연결

팀에서 사용하는 런치에서 아래 세 노드를 함께 실행한다. 이 변경에는 별도의 통합 런치나 시뮬레이션 자산을 포함하지 않는다.

| 패키지 / 실행 항목 | 역할 |
| --- | --- |
| `driving` / `driving` | 차선 추종, 주차 ROI와 속도 출력 제한 |
| `mission_model` / `detection` | P·좌회전 표지 검출, 주차 중 표지 추론 중지 |
| `parking` / `detection` | 흰선 판정, 원본 6동작, 출차 인계 |

parking을 포함해야 `/parking/lane_mode`가 발행되어 진입 ROI와 표지 중지가 적용된다. 세 노드는 같은 ROS 도메인에서 연결하고, 시뮬레이션에서는 모두 같은 `use_sim_time` 설정을 사용한다. `/camera`와 `/camera2`는 `sensor_msgs/msg/CompressedImage` 입력이다.

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
2. `APPROACH`: 왼쪽 절반 ROI의 상단을250행으로 내려 x[76,326), y[250,480)에서 차선을 추종한다. 미검출이면 저속 왼쪽 탐색, 재검출이면 기존 추종을 한다. 방향 표지판의 위쪽 확장은 사용하지 않는다. 이때부터 표지판 YOLO 추론을 중지하고 기존 True 표지 상태를 False로 해제한다.
3. `PARKING`: `/camera` 하단10%·가운데60%에서 HSV [0,0,160]~[180,40,255]의 흰 비율이60%를 초과하면 차선 속도 출력을 막고 아래6단계를 실행한다. 원본처럼 한 프레임의 조건으로 시작한다.
4. `EXIT`: 마지막 동작을 마치면 차선 주행을 재개하고 기존 왼쪽 절반 ROI(x[76,326),y[150,480))로 나간다. 진입 전용 탐색 조향을 해제한다. 표지 인식도 재개하며 대기 영상과 검출 횟수를 비우고 새로 받은 영상부터 기존 `confirm_frames`(기본3) 연속 확인을 시작한다.
5. `DONE`: 출차 후 새 `/left=True`가 오면 주차 ROI를 풀고 기존 교차로 모드와 좌회전 ROI를 적용한다. 새 노드를 띄우지 않는다. IDLE/APPROACH/PARKING 중 받은 좌회전 표지는 저장하지 않아 EXIT 진입만으로 즉시 인계하지 않는다.

| 단계 | 동작 | 선속도(m/s) | 각속도(rad/s) | 시간(초) |
|---|---|---:|---:|---:|
| 0 | 후진 | -0.2 | 0 | 1.0 |
| 1 | 회전1 | 0 | 1.2 | 1.3 |
| 2 | 전진 | 0.2 | 0 | 1.6 |
| 3 | 후진 | -0.2 | 0 | 1.6 |
| 4 | 회전2 | 0 | 1.5 | 0.9 |
| 5 | 전진 | 0.2 | 0 | 1.0 |

연결 작업 전 `construction` 브랜치의 실제 `detection.py` 수치다. `direction: left`이면 두 회전 모두 왼쪽이며 명목 회전량은 약89.4°/77.3°다. 각도 도달을 검사하지 않고0.1초 타이머로 시간을 확인한다. 각 동작 사이에0속도를 보내고 다음 tick에서 다음 동작을 시작한다. 실제 이동량·각도는 가속·제동 등의 영향을 받는다.

`config/parking_sim.yaml`에는 `direction`, `white_ratio`, `linear_speeds`, `angular_speeds`, `durations`만 있다. 패키지의 `share/parking/config`에도 설치되며, 기존 launch의 parking 노드 `parameters` 또는 실행 시 `--ros-args --params-file`로 전달한다. 파일을 지정하지 않아도 코드 기본값은 같다. 이 값들은 시작 시 적용되므로 변경 후 노드를 재시작한다. ROI의 기본 너비는 driving의 `yolo_lane.roi_half_width`, 출차 상단은 `yolo_lane.roi_top_row`를 따른다.

진입 전용 값은 `/driving`의 아래 파라미터로 조절한다. 실행 중 설정은 다음 처리 프레임부터 적용되며 재시작하면 코드 기본값으로 돌아간다. 주차 YAML이 아니라 driving 파라미터다.

```bash
ros2 param set /driving parking.approach_roi_top_row 250
ros2 param set /driving parking.search_speed 0.05
ros2 param set /driving parking.search_angular 0.8
```

상단 행을 높이면 더 가까운 영역만 본다. 각속도0.8은 약46°/s의 왼쪽 회전 속도이며 목표 회전각이 아니다. 세 값은 현재 초기 설정이며 실제 진입 성공에 맞춰 보정한 값은 아니다.

## 제어 연결

주차는 다른 미션의 속도를 중계하지 않는다. driving과 미션들이 기존 `/cmd_vel`을 직접 사용하고, 주차는 고정동작과 정지·취소 때만 속도를 발행한다. 대기·접근·출차·완료 동안 주기적으로0속도를 보내지 않는다. 다른 직접 제어 미션과 순서대로 동작하는 구조이며 동시 실행 우선순위를 중재하는 기능은 없다.

- `/lane_state` Bool: 외부 주행 허용. 주차도 구독하지만 이 토픽에 자기 정지 신호를 재발행하지 않는다.
- `/parking`, `/left` Bool: mission_model의 표지 상태. 기존 연속 검출 확인을 재사용한다.
- `/parking_state` Bool: 수동 시작 요청·취소 입력. 자기재발행하지 않는다.
- `/parking/status` String: 상태가 바뀔 때 `phase`, `step`, `reason`을 담은 JSON을 발행한다. 6동작의 각 단계마다 갱신되는 실시간 진행률은 아니다.
- `/parking/lane_mode` String: `left`는 가까운 왼쪽ROI와 미검출 탐색, `exit`는 기존 왼쪽 절반ROI와 차선 추종, `maneuver`는 검출하면서 차선 속도 출력만 중지, `halt`는 중지 유지, `intersection`은 주차 제한 해제 및 기존 교차로·좌회전 처리, 빈 문자열은 일반 상태다.

`mission_model`도 `/parking/lane_mode`를 transient local로 구독한다. `left`/`maneuver`/`halt`에서는 표지판 추론·검출 누적을 쉬고, `exit`/`intersection`/빈 문자열에서는 재개한다. 중지·재개 전환 때 이전 영상·카운터·활성 표지를 초기화하므로 계속 화면에 남아 있는 좌회전 표지도 출차 후 다시 True를 발행할 수 있다. P표지를 보기 전 일반 주행의 표지 처리는 유지된다.

외부 `/lane_state=True`는 `maneuver`/`halt`의 차선 출력을 강제로 켜지 않는다. 외부 False, 수동 취소, 잘못된 입력 영상, 진행 중 시뮬레이션 시간 역행은 HALT로 정지한다. 진행 중 노드를 정상 종료할 때도 정지한다. 종료 뒤 ROS publish나 `/parking_state` 자기구독 루프는 없다.

출구의 좌회전 인계는 한 모드 메시지로 처리해 `/left`가 driving에 먼저 도착해도 놓치지 않는다. 이후 방향 표지와 교차로 상태 메시지는 기존 방식대로 처리된다. 별도의 속도 중계 remap은 필요 없다. 주차의 상태·모드·진입 시 정지·로그는 `set_phase()` 한 곳에서 함께 갱신한다.

## 확인 범위

관련 검사는 [주차 제어](test/test_parking_control.py), [driving 연결](../../driving/test/test_parking_handoff.py), [표지 인식 중지](../mission_model/test/test_parking_sign_pause.py)에 있다.

ROS context와 모델을 대체한 검사로 흰선 ROI와 6단계 수치·종료, 외부 정지·취소, 인계 순서, 진입 ROI·탐색·재검출 복귀, 표지 중지·초기화·새 영상 재확인을 확인한다. 실제 카메라·CUDA 추론·로봇 주행 성공과는 구분한다.

기존 사용자 시험 로그에서는 P 검출 후 APPROACH와 표지 중지를 확인했지만, 흰선 조건을 통과한 PARKING 및 EXIT 기록은 없었다. 실제 흰선 ROI·비율을 계측하고 6동작·출차·교차로 인계를 검증하는 작업이 남아 있다. 다른 컴퓨터의 런치를 이용한 전체 주차 성공은 아직 검증하지 않았다.
