#!/usr/bin/env python3
"""
RFID读写器功能包测试脚本
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from rfid_reader.msg import RfidTag, RfidTagArray, RfidReaderStatus
from rfid_reader.srv import RfidCommand


class TestRFIDReader(unittest.TestCase):
    """RFID读写器测试类"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.test_node = Node('test_rfid_reader')
        
        # 创建服务客户端
        cls.command_client = cls.test_node.create_client(
            RfidCommand, 'rfid/command')
        
        # 等待服务可用
        cls.command_client.wait_for_service(timeout_sec=10.0)
        
    @classmethod
    def tearDownClass(cls):
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_service_availability(self):
        """测试服务可用性"""
        self.assertTrue(self.command_client.service_is_ready())
    
    def test_get_status_command(self):
        """测试获取状态命令"""
        request = RfidCommand.Request()
        request.command = "get_status"
        
        future = self.command_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        
        self.assertTrue(future.done())
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success)
    
    def test_invalid_command(self):
        """测试无效命令"""
        request = RfidCommand.Request()
        request.command = "invalid_command"
        
        future = self.command_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        
        response = future.result()
        self.assertFalse(response.success)
        self.assertIn("未知命令", response.message)


class TestRFIDTopics(unittest.TestCase):
    """RFID话题测试类"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.test_node = Node('test_rfid_topics')
        cls.received_tags = []
        cls.received_status = None
        
        # 创建订阅者
        cls.tags_sub = cls.test_node.create_subscription(
            RfidTagArray, 'rfid/tags', cls.tags_callback, 10)
        cls.status_sub = cls.test_node.create_subscription(
            RfidReaderStatus, 'rfid/status', cls.status_callback, 10)
    
    @classmethod
    def tearDownClass(cls):
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def tags_callback(cls, msg):
        cls.received_tags.append(msg)
    
    @classmethod
    def status_callback(cls, msg):
        cls.received_status = msg
    
    def test_topic_publishing(self):
        """测试话题发布"""
        # 等待消息
        timeout = time.time() + 10.0
        while time.time() < timeout and not self.received_status:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        
        self.assertIsNotNone(self.received_status)
        self.assertIsInstance(self.received_status, RfidReaderStatus)


if __name__ == '__main__':
    unittest.main()