# 미션 표지판 모델

- `yolo11n_mission.pt`: 원본 YOLO11n detect 가중치(8클래스, imgsz 320).
- `yolo11n_mission.onnx`: 위 모델을 `export(format='onnx', imgsz=320, opset=17, simplify=True)`로
  변환(ultralytics 8.4.192). `model_path`가 `''`이면 노드는 `.onnx`를 우선 사용하고
  onnxruntime만으로 추론한다(torch 미사용). `.pt`를 쓰려면 `model_path`로 지정한다.

`device`와 Jetson 설치 방법은 `driving/models/README.md`의 "ONNX 모델" 항목과 같다.
추론 코드 `mission_model/yolo_onnx.py`는 `driving/driving/detection/yolo_onnx.py`와 같은 파일이다.

기본 `device`는 `cuda`로, ONNX Runtime CUDA를 사용하며 TensorRT 엔진은 생성하지 않는다.
`scripts/detection`의 shebang은 `#!/home/atom5/turtlebot3_ws/.venv/bin/python`이다.
`setup.py`는 이 파일을 `lib/mission_model/detection`에 설치해 시스템 Python으로
재빌드해도 가상환경 경로를 유지한다. 다른 경로로 옮기면 shebang의 절대 경로를 수정하고 재빌드한다.

ROS와 workspace setup을 읽은 터미널에서 다음 중 하나로 실행한다.

```bash
ros2 run mission_model detection
ros2 launch mission_model mission_model.launch.py
```

launch도 기본적으로 설치된 실행 파일의 shebang을 사용한다. 별도 Python 환경을 쓰려면
`python:=/absolute/path/to/python`을 지정한다. 영상 입력은 `/camera2`다.
