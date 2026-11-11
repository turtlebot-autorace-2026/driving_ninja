"""torch/ultralytics 없이 onnxruntime으로 YOLO(detect/segment) ONNX를 추론한다.

ultralytics의 .pt 추론(letterbox → NMS → 마스크 복원)과 같은 계산을 numpy/OpenCV로 한다.
Jetson(JetPack 5, Python 3.8)에서 쓰기 위해 3.8 문법만 사용한다.

  device ''      : TensorRT → CUDA → CPU 중 설치된 provider를 순서대로 사용
                   (TensorRT는 libnvinfer가 실제로 있을 때만 넣는다)
  device 'trt'   : TensorRT(FP16, 엔진 캐시) + CUDA/CPU 대체
  device 'cuda' / '0' : CUDA + CPU 대체
  device 'cpu'   : CPU만

TensorRT provider는 첫 실행에서 엔진을 빌드한다(Orin Nano에서 수 분). 빌드한 엔진은
`trt_cache_dir`(기본 ~/.cache/yolo_trt)에 저장되어 다음 실행부터 바로 읽힌다.

mission_model/mission_model/yolo_onnx.py와 driving/driving/detection/yolo_onnx.py는 같은 파일이다. 한쪽을 고치면 다른 쪽도 맞춘다.
"""
import ast
import os

import cv2
import numpy as np


def _has_tensorrt():
    """TensorRT 라이브러리(libnvinfer)가 시스템에 있는지 확인한다.

    pip onnxruntime-gpu는 TensorRT가 없어도 TensorrtExecutionProvider를 사용 가능으로 표시한다.
    그 상태로 TensorRT를 맨 앞에 두면 세션 생성이 실패하면서 CUDA를 건너뛰고 CPU로 넘어간다.
    """
    import ctypes.util
    return ctypes.util.find_library('nvinfer') is not None


def _providers(device, cache_dir):
    import onnxruntime as ort
    available = ort.get_available_providers()
    device = (device or '').strip().lower()
    trt = ('TensorrtExecutionProvider', {
        'trt_fp16_enable': True,
        'trt_engine_cache_enable': True,
        'trt_engine_cache_path': cache_dir,
    })
    if device == 'cpu':
        wanted = ['CPUExecutionProvider']
    elif device in ('cuda', 'gpu') or device.isdigit():
        wanted = ['CUDAExecutionProvider', 'CPUExecutionProvider']
    elif device in ('trt', 'tensorrt'):
        wanted = [trt, 'CUDAExecutionProvider', 'CPUExecutionProvider']
    elif device == '':
        wanted = ([trt] if _has_tensorrt() else []) + ['CUDAExecutionProvider', 'CPUExecutionProvider']
    else:
        raise ValueError("device must be '', 'trt', 'cuda', '0' or 'cpu': " + device)
    # 명시한 가속기가 없으면 CPU로 조용히 넘어가지 않고 실패시킨다('' 자동만 대체 허용).
    required = {'trt': 'TensorrtExecutionProvider', 'tensorrt': 'TensorrtExecutionProvider',
                'cuda': 'CUDAExecutionProvider', 'gpu': 'CUDAExecutionProvider'}
    need = required.get(device, 'CUDAExecutionProvider' if device.isdigit() else None)
    if need and need not in available:
        raise ValueError(need + ' is not available (installed: ' + str(available) + ')')
    # pip onnxruntime-gpu(1.21+)는 pip로 설치된 CUDA/cuDNN(nvidia-*-cu12)을 스스로 찾지 못한다.
    # 미리 로드해 두지 않으면 CUDA provider 생성이 실패하고 CPU로 넘어간다. Jetson의 구버전에는 이 함수가 없다.
    if 'CUDAExecutionProvider' in available and hasattr(ort, 'preload_dlls'):
        ort.preload_dlls()
    return [p for p in wanted if (p[0] if isinstance(p, tuple) else p) in available]


def letterbox(img, shape):
    """ultralytics LetterBox(center=True, 114). shape=(입력 높이, 입력 너비)."""
    h, w = img.shape[:2]
    r = min(shape[0] / h, shape[1] / w)
    nw, nh = max(round(w * r), 1), max(round(h * r), 1)
    if (nw, nh) != (w, h):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (shape[1] - nw) / 2, (shape[0] - nh) / 2
    top, left = round(dh - 0.1), round(dw - 0.1)
    bottom, right = round(dh + 0.1), round(dw + 0.1)
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT,
                             value=(114, 114, 114))
    return img, (nh / h, nw / w), (left, top)


def nms(boxes, scores, iou):
    """xyxy 박스의 greedy NMS. 점수 높은 순으로 남긴 인덱스."""
    x1, y1, x2, y2 = boxes.T
    area = (x2 - x1).clip(0) * (y2 - y1).clip(0)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        rest = order[1:]
        w = (np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest])).clip(0)
        h = (np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest])).clip(0)
        inter = w * h
        order = rest[inter / (area[i] + area[rest] - inter + 1e-7) <= iou]
    return np.array(keep, dtype=np.int64)


class YoloOnnx:
    """ONNX YOLO 추론기. predict()는 박스/점수/클래스(+마스크)를 원본 이미지 좌표로 돌려준다."""

    def __init__(self, model_path, device='', trt_cache_dir=None):
        import onnxruntime as ort
        if not os.path.isfile(model_path):
            raise ValueError('ONNX model not found: ' + model_path)
        cache_dir = trt_cache_dir or os.path.expanduser('~/.cache/yolo_trt')
        os.makedirs(cache_dir, exist_ok=True)
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, opts,
                                            providers=_providers(device, cache_dir))
        meta = self.session.get_modelmeta().custom_metadata_map
        self.names = ast.literal_eval(meta['names']) if 'names' in meta else {}
        self.task = meta.get('task', 'detect')
        self.end2end = meta.get('end2end', 'False') == 'True'
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        if not all(isinstance(d, int) for d in inp.shape[2:]):
            raise ValueError('only fixed-size ONNX inputs are supported: ' + str(inp.shape))
        self.shape = (inp.shape[2], inp.shape[3])  # (h, w); 직사각 입력이면 패딩 없이 추론한다
        self.imgsz = max(self.shape)
        self.nc = len(self.names)

    @property
    def providers(self):
        return self.session.get_providers()

    def predict(self, img, conf=0.25, iou=0.7, max_det=300, imgsz=None, retina_masks=True):
        """img: BGR uint8 (H, W, 3).

        반환: dict(boxes=(N,4) xyxy 원본 좌표, scores=(N,), classes=(N,) int,
                   masks=(N,H,W) bool 또는 None(detect 모델)).
        imgsz는 ONNX 입력 크기로 고정이라 다르면 무시한다(호환용 인자).
        """
        h0, w0 = img.shape[:2]
        x, (gy, gx), (px, py) = letterbox(img, self.shape)
        x = np.ascontiguousarray(x[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        outs = self.session.run(None, {self.input_name: x})
        pred = outs[0][0]
        proto = outs[1][0] if len(outs) > 1 else None

        if self.end2end:  # (300, 6 + nm): x1,y1,x2,y2,score,cls,...
            rows = pred[pred[:, 4] > conf][:max_det]
            boxes, scores, classes, coeffs = rows[:, :4], rows[:, 4], rows[:, 5], rows[:, 6:]
        else:  # (4 + nc + nm, A): cx,cy,w,h, 클래스 점수, 마스크 계수
            pred = pred.T
            cls_scores = pred[:, 4:4 + self.nc]
            classes = cls_scores.argmax(1)
            scores = cls_scores[np.arange(len(classes)), classes]
            keep = scores > conf
            pred, scores, classes = pred[keep], scores[keep], classes[keep]
            cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
            boxes = np.stack((cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2), 1)
            coeffs = pred[:, 4 + self.nc:]
            if len(scores) > 30000:
                top = scores.argsort()[::-1][:30000]
                boxes, scores, classes, coeffs = boxes[top], scores[top], classes[top], coeffs[top]
            if len(scores):
                # 클래스별 NMS: 클래스마다 좌표를 떨어뜨려 한 번에 계산(ultralytics max_wh=7680).
                k = nms(boxes + classes[:, None] * 7680.0, scores, iou)[:max_det]
                boxes, scores, classes, coeffs = boxes[k], scores[k], classes[k], coeffs[k]

        # letterbox 좌표 → 원본 좌표
        boxes = boxes.astype(np.float32).copy()
        boxes[:, [0, 2]] = ((boxes[:, [0, 2]] - px) / gx).clip(0, w0)
        boxes[:, [1, 3]] = ((boxes[:, [1, 3]] - py) / gy).clip(0, h0)
        result = {'boxes': boxes, 'scores': scores.astype(np.float32),
                  'classes': classes.astype(np.int64), 'masks': None}
        if proto is not None:
            masks = self._masks(proto, coeffs, boxes, (h0, w0), (py, px, gy, gx))
            keep = masks.reshape(len(masks), -1).any(1) if len(masks) else np.zeros(0, bool)
            for key in ('boxes', 'scores', 'classes'):
                result[key] = result[key][keep]
            result['masks'] = masks[keep]
        return result

    def _masks(self, proto, coeffs, boxes, shape, pad):
        """ultralytics process_mask_native: 프로토타입 → 입력 크기 → 패딩 제거 → 원본 크기 → 박스 자르기."""
        h, w = shape
        n = len(coeffs)
        if n == 0:
            return np.zeros((0, h, w), dtype=bool)
        c, mh, mw = proto.shape
        logits = (coeffs.astype(np.float32) @ proto.reshape(c, -1)).reshape(n, mh, mw)
        py, px, gy, gx = pad
        out = np.empty((n, h, w), dtype=bool)
        rows = np.arange(h, dtype=np.float32)[:, None]
        cols = np.arange(w, dtype=np.float32)[None, :]
        # cv2.resize는 채널 수 제한(512)이 있어 마스크를 묶음 단위로 처리한다.
        for s in range(0, n, 256):
            chunk = logits[s:s + 256].transpose(1, 2, 0)
            up = cv2.resize(chunk, (self.shape[1], self.shape[0]), interpolation=cv2.INTER_LINEAR)
            up = up.reshape(self.shape[0], self.shape[1], -1)
            bottom, right = py + max(round(h * gy), 1), px + max(round(w * gx), 1)
            up = cv2.resize(np.ascontiguousarray(up[py:bottom, px:right]), (w, h),
                            interpolation=cv2.INTER_LINEAR).reshape(h, w, -1)
            out[s:s + 256] = (up > 0).transpose(2, 0, 1)
        for i, (x1, y1, x2, y2) in enumerate(boxes):
            out[i] &= (cols >= x1) & (cols < x2) & (rows >= y1) & (rows < y2)
        return out
