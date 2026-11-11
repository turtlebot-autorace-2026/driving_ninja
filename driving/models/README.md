# YOLO 차선 모델의 출처

- 원본 저장소: https://github.com/turtlebot-autorace-2026/lane_detection_dl
- 기준 커밋: `f1dcce74456cb0eea68c4db89346cd02137d440b`
- 원본 경로: `lane_detection/models/lane_yolo26n_seg.pt`
- SHA256: `b7afc9eec9ec74c475a4227d4acb780288028568d3530db1ce3b763f2fc3bcd6`
- 클래스: `0 = white`, `1 = yellow`

이 파일은 원본 가중치를 변경 없이 복사한 것이다. `setup.py`는 모델을
`share/driving/models`에 설치한다. 노드의 `yolo_lane.model_path` 시작 파라미터로
다른 모델 경로를 지정할 수 있지만 클래스 의미와 출력 마스크 형식을 맞춰야 한다.

`driving/detection/lane_estimator.py`와 `driving/steering/pure_pursuit.py`의
기하 및 조향 계산도 위 커밋을 기준으로 이식했다. 검출기의 `detect()`는 원본
`detection.py`의 추론 계산을 유지한다. 검출/조향 호출을 분리했으며 외부 차선
enable 토픽(`/lane_state`)은 공통 노드가 처리한다. 초기 False에서도 검출·조향 계산과 디버그 발행을 계속하고, True로 전환한 뒤 새로 수신한 영상부터 주행 속도를 발행한다. 교차로 상태(`/intersection_state`)와
최대 속도·곡률 배율 정책은 원본대로 연결한다. 최대 속도 파라미터 이름은
`pure_pursuit.v_max_intersection`이며 원본 기본 0.12m/s(현재 노드 기본 0.30m/s), 내부 곡률 배율은 0.7이다.

## ONNX 모델 (Jetson 기본값)

노드 기본 모델은 `lane_yolo26n_seg.onnx`다. `.onnx`는 `driving/detection/yolo_onnx.py`가
onnxruntime만으로 추론해 torch/ultralytics를 import하지 않는다(PC CPU 기준 피크 RAM
약 800MB → 약 215MB). `yolo_lane.model_path`에 `.pt`를 주면 기존 ultralytics 경로를 쓴다.

- 변환: `YOLO('lane_yolo26n_seg.pt').export(format='onnx', imgsz=(480, 640), opset=17, simplify=True)`
  (ultralytics 8.4.192). BEV가 640x480이라 직사각 입력으로 내보내 `.pt` 추론과 마스크가 같다.
  입력 크기가 ONNX에 고정되므로 `yolo_lane.imgsz`는 `.onnx`에서 쓰이지 않는다.
- `yolo_lane.device` 기본값: `'cuda'`(ONNX Runtime CUDA, TensorRT 엔진 생성 없음).
  `'cpu'`, `'0'`, `'trt'`, `''` 자동(TensorRT → CUDA → CPU)도 지정할 수 있다.
- Jetson Orin Nano(JetPack 5, Python 3.8): Jetson Zoo의 JetPack 5용 `onnxruntime-gpu` 휠을
  설치한다(pip의 일반 `onnxruntime`은 CPU 전용). `'trt'` 또는 `''` 자동 선택으로
  TensorRT provider를 사용할 때는 첫 실행에서 FP16 엔진을
  빌드하고(수 분 걸림) `~/.cache/yolo_trt`에 저장해 다음 실행부터 재사용한다.
  모델을 다시 변환하면 이 캐시를 지운다.
