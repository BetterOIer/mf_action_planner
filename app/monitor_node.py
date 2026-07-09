#!/usr/bin/env python3
"""
中继节点：限频转发 ROS Topic 数据，提供 HTTP API 给网页。
GET  /data   → 返回最新 Topic 数据的 JSON
POST /action → 接收网页动作请求，调用 ROS Service/Action/Topic
"""
import json
import math
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Int32

from r2_interfaces.srv import StartAutonomy
from action_of_motion_interfaces.action import MoveToPose


DATA_TIMEOUT_SEC = 1.0


# ================================================================
#  HTTP Request Handler
# ================================================================
class _Handler(BaseHTTPRequestHandler):
    node: 'MonitorNode' = None  # injected by MonitorNode

    def do_GET(self):
        if self.path == '/data':
            data = self.node.get_data()
            self._json(data)
        else:
            self.send_response(404); self.end_headers()

    def do_POST(self):
        if self.path == '/action':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8')
            self.node.handle_action(body)
            self._json({'ok': True})
        else:
            self.send_response(404); self.end_headers()

    def _json(self, obj):
        payload = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        pass  # suppress HTTP access logs


# ================================================================
#  Monitor Node
# ================================================================
class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # ---- 存储最新数据 ----
        self._reloc_latest = None   # PoseStamped
        self._sensor_latest = None  # Float32MultiArray
        self._r0_latest = None      # Float32MultiArray
        self._aruco_latest = None   # Int32
        self._topic_last_seen = {
            'reloc': None,
            'sensors': None,
            'r0x0121': None,
            'aruco': None,
        }
        self._data_lock = threading.Lock()

        # ---- 订阅原始 Topic ----
        self.create_subscription(PoseStamped, '/odin1/relocation', self._reloc_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensor_distances', self._sensor_cb, 10)
        self.create_subscription(Float32MultiArray, '/r0x0121', self._r0_cb, 10)
        self.create_subscription(Int32, '/aruco_comm/tx_id', self._aruco_cb, 10)

        # ---- ROS 接口客户端 ----
        self._start_cli = self.create_client(StartAutonomy, '/bt_engine/start_autonomy')
        self._move_cli = ActionClient(self, MoveToPose, '/move_to_pose')
        self._lift_pub = self.create_publisher(Float32MultiArray, 't0x0112_action', 10)

        # ---- 启动 HTTP server ----
        _Handler.node = self
        self._httpd = HTTPServer(('0.0.0.0', 8765), _Handler)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info('MonitorNode ready on :8765')

    # ---- Topic 回调 ----
    def _mark_seen(self, topic_name: str):
        self._topic_last_seen[topic_name] = time.monotonic_ns()

    def _reloc_cb(self, msg):
        with self._data_lock:
            self._reloc_latest = msg
            self._mark_seen('reloc')

    def _sensor_cb(self, msg):
        with self._data_lock:
            self._sensor_latest = msg
            self._mark_seen('sensors')

    def _r0_cb(self, msg):
        with self._data_lock:
            self._r0_latest = msg
            self._mark_seen('r0x0121')

    def _aruco_cb(self, msg):
        with self._data_lock:
            self._aruco_latest = msg
            self._mark_seen('aruco')

    def _topic_online(self, last_seen_ns, now_ns: int) -> bool:
        if last_seen_ns is None:
            return False
        return (now_ns - last_seen_ns) <= int(DATA_TIMEOUT_SEC * 1e9)

    # ---- GET /data ----
    def get_data(self):
        result = {}
        now_ns = time.monotonic_ns()
        with self._data_lock:
            reloc_latest = self._reloc_latest
            sensor_latest = self._sensor_latest
            r0_latest = self._r0_latest
            aruco_latest = self._aruco_latest
            last_seen = dict(self._topic_last_seen)

        online = {
            name: self._topic_online(seen_ns, now_ns)
            for name, seen_ns in last_seen.items()
        }
        result['online'] = online

        # relocation: PoseStamped → {x_mm, y_mm, yaw_deg}
        if online['reloc'] and reloc_latest is not None:
            p = reloc_latest.pose.position
            q = reloc_latest.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            result['reloc'] = {
                'x': round(p.x * 1000.0, 1),      # m→mm
                'y': round(p.y * 1000.0, 1),
                'yaw_deg': round(yaw * 180.0 / math.pi, 2),
            }

        # sensor_distances: Float32MultiArray
        if online['sensors'] and sensor_latest is not None:
            result['sensors'] = [round(v, 0) for v in sensor_latest.data[:8]]

        # r0x0121: Float32MultiArray
        if online['r0x0121'] and r0_latest is not None:
            result['r0x0121'] = [round(v, 3) for v in r0_latest.data[:11]]

        # aruco: Int32
        if online['aruco'] and aruco_latest is not None:
            result['aruco'] = aruco_latest.data

        return result

    # ---- POST /action ----
    def handle_action(self, body: str):
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON: {body}')
            return

        action = data.get('action', '')
        if action == 'start':
            self._handle_start(data)
        elif action == 'move':
            self._handle_move(data)
        elif action == 'lift':
            self._handle_lift(data)
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def _handle_start(self, data: dict):
        if not self._start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('start_autonomy service not available')
            return
        req = StartAutonomy.Request()
        region = data.get('region', 'full')
        req.region = region
        indices = data.get('prepare_pickup_indices', [])
        if not isinstance(indices, list):
            indices = []
        req.prepare_pickup_indices = [
            int(v) for v in indices
            if isinstance(v, (int, float)) and int(v) == v
        ]
        fut = self._start_cli.call_async(req)
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f'Start OK: {f.result().message}' if f.result().success
                else f'Start FAIL: {f.result().message}')
        )

    def _handle_move(self, data: dict):
        goal = MoveToPose.Goal()
        goal.pid_profile = 1
        goal.x = float(data.get('x', 0.0))
        goal.y = float(data.get('y', 0.0))
        goal.yaw_deg = float(data.get('yaw', 0.0)) * 180.0 / math.pi
        self._move_cli.send_goal_async(goal)
        self.get_logger().info(
            f'Move: x={goal.x:.3f} y={goal.y:.3f} yaw={goal.yaw_deg:.1f}deg')

    def _handle_lift(self, data: dict):
        msg = Float32MultiArray()
        msg.data = [
            float(data.get('fl', 0.0)),
            float(data.get('fr', 0.0)),
            float(data.get('rl', 0.0)),
            float(data.get('rr', 0.0)),
        ]
        self._lift_pub.publish(msg)
        self.get_logger().info(f'Lift: {msg.data}')


def main():
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._httpd.shutdown()
    node.destroy_node()
    rclpy.shutdown()
