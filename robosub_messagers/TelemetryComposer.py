import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.node import Node

from gps_interfaces.msg import Coordinates
from rfm9x_interfaces.msg import Payload, ID

from robosub_messagers.robosub_pb2 import TelemetryMessage


class TelemetryComposer(Node):
    """Receives and logs data to send to the controller.

    This node records telemetry data for later retrieval. Upon request,
    it serializes the data into a message for the controller.
    """

    def __init__(self, executor: Executor):
        super().__init__(node_name='telemetry_composer', parameter_overrides=[])

        self.log = self.get_logger()
        self.executor: Executor = executor
        self._sub_callback_group = ReentrantCallbackGroup()
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()

        # Resend request subscription (from CommandParser)
        self._resend_sub = self.create_subscription(ID, 'resend_request',
                self._resend_sub_callback, 1, callback_group=self._sub_callback_group)
        # Data subscriptions
        self._coord_sub = self.create_subscription(Coordinates, 'coordinates',
                self._coord_sub_callback, 1, callback_group=self._sub_callback_group)
        '''
        self.depth_client = self.create_client(Depth, 'depth',
            callback_group=self.parallel_client_group)
        self.battery_client = self.create_client(Battery, 'battery',
            callback_group=self.parallel_client_group)
        self.roll_client = self.create_client(Roll, 'aroll',
            callback_group=self.attitude_client_group)
        self.pitch_client = self.create_client(Pitch, 'apitch',
            callback_group=self.attitude_client_group)
        self.yaw_client = self.create_client(Yaw, 'ayaw',
            callback_group=self.attitude_client_group)
        '''

        # Store sequence of send messages
        self._msg_id = 0
        self.telem = TelemetryMessage()
        # Publish telemetry data for radio controller
        self._telem_pub = self.create_publisher(Payload, 'telem_data', 1)
        # ...at intervals of 2 seconds
        self._pub_timer = self.create_timer(2.0, self._pub_timer_callback, self._timer_callback_group)

    def _resend_sub_callback(self, msg):
        self.telem.requesting_resend = True
        self.telem.resend = msg.id

    def _coord_sub_callback(self, msg):
        self.telem.position.latitude = msg.lat
        self.telem.position.longitude = msg.lon

    def _pose_sub_callback(self, msg):
        self.telem.pose.pitch = msg.pitch
        self.telem.pose.roll = msg.roll
        self.telem.pose.yaw = msg.yaw

    def _bat_sub_callback(self, msg):
        self.telem.battery = msg.battery

    def _pub_timer_callback(self):
        self.telem.id = self._msg_id
        self._msg_id += 1

        telem_data = [byte for byte in self.telem.SerializeToString()]
        self._telem_pub.publish(Payload(payload=telem_data))
        self.log.info(f'Published:\n{self.telem}')

        # Reset resend flag
        self.telem.requesting_resend = False

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = TelemetryComposer(executor)
    executor.add_node(node)

    node.get_logger().info('Running')
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
