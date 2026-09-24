#!/usr/bin/env python3
"""
Kratos Camera GUI — Flask web interface for camera control and feed viewing.

Runs on the operator laptop. Connects to the ROS 2 camera_stream_node
running on the robot laptop via DDS, and receives UDP H264 streams
which are re-served as MJPEG for browser viewing.

Usage:
    source /opt/ros/<distro>/setup.bash
    source ~/your_ros2_ws/install/setup.bash
    pip install flask
    python3 app.py
"""

import threading
import time
import signal
import sys

from flask import Flask, render_template, jsonify, Response

# ---------------------------------------------------------------------------
# GStreamer init (must come before any Gst import usage)
# ---------------------------------------------------------------------------
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
Gst.init(None)

# ---------------------------------------------------------------------------
# ROS 2 imports (requires sourced ROS 2 workspace with kratos_msgs built)
# ---------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from kratos_msgs.msg import CameraStreamList
from kratos_msgs.srv import (
    StartCameraStream,
    StopCameraStream,
    StartAllCameras,
    StopAllCameras,
)

# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------
app = Flask(__name__)


# ===========================================================================
# Stream Receiver — receives UDP H264 and provides JPEG frames
# ===========================================================================
class StreamReceiver:
    """Receives a single UDP H264 RTP stream and decodes to JPEG frames."""

    def __init__(self, port):
        self.port = port
        self.pipeline = None
        self.appsink = None
        self._running = False
        self._start_pipeline()

    def _start_pipeline(self):
        pipeline_str = (
            f'udpsrc port={self.port} '
            f'! application/x-rtp,encoding-name=H264,payload=96 '
            f'! rtph264depay ! h264parse ! avdec_h264 '
            f'! videoconvert '
            f'! jpegenc quality=80 '
            f'! appsink name=sink emit-signals=false sync=false '
            f'max-buffers=2 drop=true'
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.pipeline.set_state(Gst.State.PLAYING)
        self._running = True

    def get_frame(self):
        """Pull one JPEG frame (blocking up to 1 s). Returns bytes or None."""
        if not self._running or self.appsink is None:
            return None
        sample = self.appsink.try_pull_sample(Gst.SECOND)
        if sample is None:
            return None
        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return None
        data = bytes(map_info.data)
        buf.unmap(map_info)
        return data

    def stop(self):
        """Tear down the GStreamer pipeline."""
        self._running = False
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsink = None


# ===========================================================================
# ROS 2 GUI Node
# ===========================================================================
class GuiNode(Node):
    """Lightweight ROS 2 node that subscribes to camera info and calls services."""

    def __init__(self):
        super().__init__('kratos_gui_node')
        self._cameras = []
        self._lock = threading.Lock()

        # Topic subscription — camera list
        self.create_subscription(
            CameraStreamList,
            '/kratos/available_cameras',
            self._on_cameras,
            10,
        )

        # Service clients
        self.start_client = self.create_client(
            StartCameraStream, '/kratos/cameras/start_stream'
        )
        self.stop_client = self.create_client(
            StopCameraStream, '/kratos/cameras/stop_stream'
        )
        self.start_all_client = self.create_client(
            StartAllCameras, '/kratos/cameras/start_all'
        )
        self.stop_all_client = self.create_client(
            StopAllCameras, '/kratos/cameras/stop_all'
        )

        self.get_logger().info('GUI node initialised — waiting for camera data …')

    # -- Topic callback -----------------------------------------------------
    def _on_cameras(self, msg):
        cameras = []
        for stream in msg.camera_streams:
            cameras.append({
                'device': stream.camera_device,
                'name': stream.camera_name,
                'resolution': stream.resolution,
                'fps': stream.fps,
                'port': stream.port,
                'active': stream.active,
            })
        with self._lock:
            self._cameras = cameras

    def get_cameras(self):
        with self._lock:
            return list(self._cameras)

    # -- Service helpers ----------------------------------------------------
    def call_start(self, camera_name):
        req = StartCameraStream.Request()
        req.camera_name = camera_name
        return self._call_service(self.start_client, req)

    def call_stop(self, camera_name):
        req = StopCameraStream.Request()
        req.camera_name = camera_name
        return self._call_service(self.stop_client, req)

    def call_start_all(self):
        req = StartAllCameras.Request()
        return self._call_service(self.start_all_client, req)

    def call_stop_all(self):
        req = StopAllCameras.Request()
        return self._call_service(self.stop_all_client, req)

    def _call_service(self, client, request, timeout=5.0):
        """Call a service asynchronously and wait for the result."""
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=2.0)
            if not client.service_is_ready():
                return {'success': False, 'message': 'Service not available'}

        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)

        if future.done():
            result = future.result()
            return {'success': result.success, 'message': result.message}
        return {'success': False, 'message': 'Service call timed out'}


# ===========================================================================
# Globals
# ===========================================================================
gui_node = None

_receivers = {}          # port → StreamReceiver
_receivers_lock = threading.Lock()


def _get_receiver(port):
    """Get or create a StreamReceiver for the given UDP port."""
    with _receivers_lock:
        if port not in _receivers:
            _receivers[port] = StreamReceiver(port)
        return _receivers[port]


def _cleanup_receiver(port):
    """Stop and remove a StreamReceiver."""
    with _receivers_lock:
        receiver = _receivers.pop(port, None)
    if receiver is not None:
        receiver.stop()


def _cleanup_all_receivers():
    """Stop and remove all StreamReceivers."""
    with _receivers_lock:
        ports = list(_receivers.keys())
    for port in ports:
        _cleanup_receiver(port)


# ===========================================================================
# Flask Routes
# ===========================================================================

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/cameras')
def api_cameras():
    """Return JSON list of all discovered cameras and their status."""
    if gui_node is None:
        return jsonify([])
    return jsonify(gui_node.get_cameras())


@app.route('/api/cameras/start/<camera_name>', methods=['POST'])
def api_start_camera(camera_name):
    if gui_node is None:
        return jsonify({'success': False, 'message': 'ROS 2 not connected'})
    result = gui_node.call_start(camera_name)
    return jsonify(result)


@app.route('/api/cameras/stop/<camera_name>', methods=['POST'])
def api_stop_camera(camera_name):
    if gui_node is None:
        return jsonify({'success': False, 'message': 'ROS 2 not connected'})
    result = gui_node.call_stop(camera_name)
    # Clean up the stream receiver for this camera
    for cam in gui_node.get_cameras():
        if cam['name'] == camera_name:
            _cleanup_receiver(cam['port'])
            break
    return jsonify(result)


@app.route('/api/cameras/start_all', methods=['POST'])
def api_start_all():
    if gui_node is None:
        return jsonify({'success': False, 'message': 'ROS 2 not connected'})
    return jsonify(gui_node.call_start_all())


@app.route('/api/cameras/stop_all', methods=['POST'])
def api_stop_all():
    if gui_node is None:
        return jsonify({'success': False, 'message': 'ROS 2 not connected'})
    result = gui_node.call_stop_all()
    _cleanup_all_receivers()
    return jsonify(result)


@app.route('/api/stream/<int:port>')
def video_stream(port):
    """MJPEG stream endpoint — browser points an <img> tag here."""
    def generate():
        receiver = _get_receiver(port)
        while True:
            frame = receiver.get_frame()
            if frame is not None:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                )
            else:
                # No frame yet — short sleep to avoid busy loop
                time.sleep(0.033)

    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )


# ===========================================================================
# ROS 2 background thread
# ===========================================================================

def _ros2_spin():
    """Initialise rclpy and spin the GUI node in a background thread."""
    global gui_node
    rclpy.init()
    gui_node = GuiNode()
    try:
        rclpy.spin(gui_node)
    except Exception:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


# ===========================================================================
# Entry point
# ===========================================================================

def _shutdown_handler(sig, frame):
    """Graceful shutdown on Ctrl-C."""
    print('\nShutting down …')
    _cleanup_all_receivers()
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, _shutdown_handler)

    # Start ROS 2 in background
    ros_thread = threading.Thread(target=_ros2_spin, daemon=True)
    ros_thread.start()
    time.sleep(1)  # Give rclpy a moment to initialise

    print('=' * 60)
    print('  Kratos Camera GUI')
    print('  Open http://localhost:5000 in your browser')
    print('=' * 60)

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
