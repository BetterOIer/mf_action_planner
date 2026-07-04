#!/usr/bin/env python3
"""
MF Action Sequence 暂存区节点

- 订阅 /mf_action_seq (Float32MultiArray)，暂存最新数据
- 新数据覆盖旧数据
- 通过 /get_action_seq 服务供 bt_engine_node 拉取，拉取后清空
- 发布 /mf_buffer_status (String) 供网页显示暂存区状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from r2_interfaces.srv import GetActionSeq


class MfBufferNode(Node):
    def __init__(self):
        super().__init__('mf_buffer_node')

        # 暂存数据
        self._stored_data = Float32MultiArray()
        self._has_data = False

        # 订阅 /mf_action_seq
        self._sub = self.create_subscription(
            Float32MultiArray,
            '/mf_action_seq',
            self._action_seq_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )

        # 服务：供 bt_engine_node 拉取
        self._srv = self.create_service(
            GetActionSeq, '/get_action_seq', self._get_action_seq_callback
        )

        # 状态发布：供网页订阅
        self._status_pub = self.create_publisher(String, '/mf_buffer_status', 10)

        # 初始状态
        self._publish_status()

        self.get_logger().warn('MF Buffer Node started — waiting for /mf_action_seq')

    def _publish_status(self):
        msg = String()
        msg.data = 'has_data' if self._has_data else 'empty'
        self._status_pub.publish(msg)

    def _action_seq_callback(self, msg: Float32MultiArray):
        """收到新的 action sequence → 覆盖暂存"""
        self._stored_data = msg
        self._has_data = True
        self._publish_status()
        steps = len(msg.data) // 8 if msg.data else 0
        self.get_logger().warn(f'Stored action seq: {steps} steps')

    def _get_action_seq_callback(self, request, response):
        """bt_engine_node 拉取 → 返回暂存数据并清空"""
        if self._has_data:
            response.data = self._stored_data
            response.has_data = True
            self._stored_data = Float32MultiArray()
            self._has_data = False
            self._publish_status()
            steps = len(response.data.data) // 8 if response.data.data else 0
            self.get_logger().warn(f'Served {steps} steps — buffer cleared')
        else:
            response.has_data = False
            self.get_logger().debug('Service called but buffer is empty')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MfBufferNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
