#!/usr/bin/env python3
"""
rx_monitor.py
订阅 serial_send 节点发布的裁判系统 RX 话题，2Hz 聚合打印，便于快速肉眼核验 MCU → NUC 数据链路。
    /referee/game_time    (Float64)   比赛剩余时间
    /referee/hp           (Float64)   机器人血量 (0-400)
    /referee/rune         (UInt8)     大符标志 (1=去大符, 0=回驻守)
    /referee/protect      (UInt8)     保护标志 (1=去高地, 0=解除)
    /referee/ammo         (Float64)   低弹量告警 (1.0=<20 需补弹, 0.0=≥20 充足)
打印格式与 serial_send.h 里原 [RX] 日志一致。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float64


class RxMonitor(Node):
    def __init__(self):
        super().__init__('rx_monitor')

        self.game_time   = None
        self.hp          = None
        self.rune        = None
        self.protect     = None
        self.ammo        = None

        self.create_subscription(Float64, '/referee/game_time',   self._cb_time,    10)
        self.create_subscription(Float64, '/referee/hp',          self._cb_hp,      10)
        self.create_subscription(UInt8,   '/referee/rune',        self._cb_rune,    10)
        self.create_subscription(UInt8,   '/referee/protect',     self._cb_protect, 10)
        self.create_subscription(Float64, '/referee/ammo',        self._cb_ammo,    10)

        self.timer = self.create_timer(0.5, self._print)
        self.get_logger().info('RxMonitor 启动: 2Hz 打印 /referee/* 五路话题')
        self.get_logger().info('"---" 表示尚未收到该话题消息（通常意味着 MCU 未发送 0xBB 帧）')

    def _cb_time(self, msg):    self.game_time   = int(msg.data)
    def _cb_hp(self, msg):      self.hp          = int(msg.data)
    def _cb_rune(self, msg):    self.rune        = int(msg.data)
    def _cb_protect(self, msg): self.protect     = int(msg.data)
    def _cb_ammo(self, msg):    self.ammo        = int(msg.data)

    @staticmethod
    def _fmt(v):
        return '---' if v is None else str(v)

    def _print(self):
        self.get_logger().info(
            f'[RX] time={self._fmt(self.game_time)} '
            f'hp={self._fmt(self.hp)} '
            f'rune={self._fmt(self.rune)} '
            f'protect={self._fmt(self.protect)} '
            f'ammo={self._fmt(self.ammo)}'
        )


def main():
    rclpy.init()
    node = RxMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
