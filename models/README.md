# Vision models

Vendored so the gateway works straight after `scripts/deploy_puppet.sh` — no
model download step on the robot, and no network dependency at startup.

| File | Used by | Source |
|---|---|---|
| `face_detection_yunet_2023mar.onnx` | `vision.detector.face_detector: yunet` | [OpenCV Zoo](https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet), 2023-mar revision. 227 KB. |

Carried over from the ROS `maki_vision` package, which resolved the same file
out of its ament share directory.

`vision.detector.yunet_model_path` in `config/puppet.yaml` overrides the
location; left empty, the gateway resolves this directory automatically.
