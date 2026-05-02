# PID Tuner GUI

A PyQt5 desktop tool for live tuning of PID controllers on an autonomous underwater vehicle (AUV). Built for the Mira2 platform at Dreadnought Robotics.

Replaces a clunky three-terminal workflow (separate ArUco tracker, C++ PID tuner, and keyboard publisher) with one self-contained app. The operator picks an axis, slides Kp/Ki/Kd, and watches the vehicle settle — with live video, error/PWM plots, and centre-detection feedback all in one window.

## Features

- **Three control modes**
  - *Lateral* — translates the vehicle left/right to centre an ArUco marker (camera X axis)
  - *Forward* — drives the vehicle towards/away from a marker using the true range from `solvePnP` (camera Z axis)
  - *Pipeline* — subscribes to a `geometry_msgs/Point` centroid topic from any perception node and drives both lateral + forward PIDs simultaneously to the pixel centre
- **Depth control** runs orthogonally to whichever horizontal mode is active — set a target pressure (mbar) and tune Kp/Ki/Kd against the live `external_pressure` telemetry
- **Yaw lock** holds the heading first received from telemetry (optional)
- **Embedded RTSP video** via a low-latency GStreamer `appsink` pipeline (PyGObject), not OpenCV's FFmpeg backend
- **Internal ArUco detection** with underwater image enhancement (bilateral filter + CLAHE + sharpen) so the GUI can use the true forward range, fixing a bug in the legacy C++ tuner that read camera-Y as range
- **Live plots** of lateral and forward PWM commanded to `/master/commands` (pyqtgraph, rolling window)
- **Centre-detection** with configurable tolerance and dwell time — reports "centred in X.XXs" once the error stays inside the band long enough
- **Slider + textbox sync** for every gain so coarse and fine adjustments are both one-click
- **Stale-pose safety**: holds neutral PWM (1500) and warns if the marker disappears for more than 0.5 s
- **Integrator reset** on every disarm (matches the legacy tuner behaviour)

## Architecture

The file is one module split into seven sections (A–G), each with a banner comment so debugging is "scroll to the section":

| Section | Class | Purpose |
|---|---|---|
| A | `ArucoDetector` | OpenCV ArUco detection + `solvePnP` + image enhancement |
| B | `PIDController` | Trapezoidal-integration PID, ported from a C++ implementation |
| C | `GstCapture` / `VideoGrabber` (QThread) | GStreamer `appsink` capture with frame-drain for freshness |
| D | `TunerNode` (rclpy.Node) | ROS2 publisher + subscribers, runs in its own spin thread |
| E | `ControlEngine` (QObject) | 100 Hz QTimer that binds detector → PID → publisher |
| F | `TunerWindow` (QMainWindow) | All Qt widgets and signal wiring |
| G | `main()` | Boot order: rclpy → Gst → QApplication |

### Threading model

```
GStreamer appsink ──▶ VideoGrabber (QThread) ──▶ Qt main thread (display + detect)
                                                          │
ROS2 spin thread ─── /master/telemetry ──┐                │
                  └─ /vision/centroid ───┤                │
                                         ▼                ▼
                                     ControlEngine (QTimer 100 Hz) ──▶ rclpy.publish() ──▶ /master/commands
```

Cross-thread data passes through `deque(maxlen=N)` ring buffers (single-producer/single-consumer is GIL-safe) and Qt's queued signal/slot connections.

### Boot order matters

`rclpy.init()` must run **before** `Gst.init()`, which must run **before** PyQt5 loads. GStreamer's init grabs boost::interprocess shared memory, which conflicts with FastDDS's transport. Switching the RMW to CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) sidesteps the conflict — set automatically at the top of the file.

## Dependencies

```
PyQt5>=5.15
pyqtgraph>=0.13
opencv-python
numpy
scipy
PyGObject  # for GStreamer
```

ROS2 side (Jazzy):
- `rclpy`
- `geometry_msgs`
- `custom_msgs` — a project-specific package providing `Commands` (8× int16 PWM channels + arm/mode strings) and `Telemetry` (heading, external_pressure, …). Drop-in replacement with your own message types is straightforward.

System packages:
- `gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav`
- `python3-gi gir1.2-gst-plugins-base-1.0`

## Status

This is a **showcase extract** from the Mira2 AUV firmware. It depends on ROS2 message types from the parent workspace and won't run standalone without those — the value here is in the architecture, threading model, and Qt/ROS/GStreamer integration patterns.
