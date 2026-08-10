# Test images — provenance

| File | Origin | Licence |
|---|---|---|
| `bus.jpg` | Standard sample image from the [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) repository | AGPL-3.0 |
| `zidane.jpg` | Standard sample image from the [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) repository | AGPL-3.0 |

Neither image was produced by this project. They are used as fixtures because
they are the canonical detector regression images — a change in detector
behaviour shows up against them immediately and comparably to everyone else's
results.

`zidane.jpg` earns its place for a second reason worth recording: its subject
stands with both arms extended, giving a bounding box of 993 x 511 px. That is
the counterexample which forced distance estimation in
`vision/pursuit/control.py` to key on box **height** and never width — a person
does not get taller when they gesture, but they do get much wider. The test
`test_distance_uses_height_not_width` pins that behaviour to this image.

## Models

No model weights are stored in this repository. `vision/fetch_models.py`
downloads them and verifies each one by loading the graph in ONNX Runtime
before accepting it:

| Model | Source | Licence |
|---|---|---|
| `yolo11n.onnx` | Ultralytics YOLO11 nano | AGPL-3.0 |
| `det_10g.onnx` (SCRFD) | InsightFace `buffalo_l` pack | Non-commercial research use |
| `w600k_r50.onnx` (ArcFace) | InsightFace `buffalo_l` pack | Non-commercial research use |
| `person_reid.onnx` | OpenCV Zoo, youtu-reid | Apache-2.0 |

**The InsightFace `buffalo_l` models are licensed for non-commercial research
use only.** That is a real constraint on this project, not a formality: it
means PURSUIT-1 as built is a research and coursework artifact and could not be
shipped as a product without replacing the face stage. Recorded here so the
limitation is visible rather than discovered later.
