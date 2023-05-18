import socket
import pickle

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point


class SocketTranslator(Node):

    def __init__(self):
        super().__init__(node_name='socket_translator', parameter_overrides=[])
        self.log = self.get_logger()

        self.SOCKET_PORT = 10000
        self._connected = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.bind(('', self.SOCKET_PORT))
        self._sock_timer = self.create_timer(0.25, self._sock_timer_callback, MutuallyExclusiveCallbackGroup())

        self._out_data = {}
        self._waypoint_sub = self.create_subscription(Point, 'waypoint',
            lambda msg: self._out_data.update(waypoint=point_to_tuple(msg)), 10)
        self._currentGPSPosition_sub = self.create_subscription(Point, 'currentGPSPosition',
            lambda msg: self._out_data.update(currentGPSPosition=point_to_tuple(msg)), 10)

        self._in_data = {}
        self._actualPitch_pub = self.create_publisher(Float64, 'actualPitch', 10)
        self._actualRoll_pub = self.create_publisher(Float64, 'actualRoll', 10)
        self._actualYaw_pub = self.create_publisher(Float64, 'actualYaw', 10)
        self._depth_pub = self.create_publisher(Float64, 'depth', 10)
        self._requestNewWaypoint_pub = self.create_publisher(Bool, 'requestNewWaypoint', 10)
        
    def _sock_timer_callback(self):
        if self._connected:
            self._relay_data()
        else:
            # Attempt to connect to sub
            srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv_sock.settimeout(0.5)
            srv_sock.bind(('', self.SOCKET_PORT))
            srv_sock.listen(1)
            try:
                self._sock, addr = srv_sock.accept()
                self._sock.settimeout(0.25)
                self._connected = True
                self.log.info('Socket connected')
            except socket.timeout:
                self._connected = False

    def _relay_data(self):
        # Receive incoming data
        try:
            self._in_data = pickle.loads(self._sock.recv(2048))
            if 'actualPitch' in self._in_data:
                self._actualPitch_pub.publish(Float64(data=self._in_data['actualPitch']))
            if 'actualRoll' in self._in_data:
                self._actualRoll_pub.publish(Float64(data=self._in_data['actualRoll']))
            if 'actualYaw' in self._in_data:
                self._actualYaw_pub.publish(Float64(data=self._in_data['actualYaw']))
            if 'depth' in self._in_data:
                self._depth_pub.publish(Float64(data=self._in_data['depth']))
            if 'requestNewWaypoint' in self._in_data:
                self._requestNewWaypoint_pub.publish(Bool())
        except socket.timeout:
            pass
        except pickle.PickleError:
            self.log.error('Pickling error')
            
        # Send outgoing data
        try:
            self._sock.sendall(pickle.dumps(self._out_data))
            self._out_data = {}
        except socket.timeout:
            pass
        except pickle.PickleError:
            self.log.error('Pickling error')

        
def point_to_tuple(p): return (p.x, p.y, p.z)

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = SocketTranslator()

    node.get_logger().info('Running')
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
