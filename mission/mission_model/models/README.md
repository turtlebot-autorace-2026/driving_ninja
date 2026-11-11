# 미션 표지판 모델

- `yolo11n_mission.pt`: 원본 YOLO11n detect 가중치(8클래스, imgsz 320).
- `yolo11n_mission.onnx`: 위 모델을 `export(format='onnx', imgsz=320, opset=17, simplify=True)`로
  변환(ultralytics 8.4.192). `model_path`가 `''`이면 노드는 `.onnx`를 우선 사용하고
  onnxruntime만으로 추론한다(torch 미사용). `.pt`를 쓰려면 `model_path`로 지정한다.

`device`와 Jetson 설치 방법은 `driving/models/README.md`의 "ONNX 모델" 항목과 같다.
추론 코드 `mission_model/yolo_onnx.py`는 `driving/driving/detection/yolo_onnx.py`와 같은 파일이다.
