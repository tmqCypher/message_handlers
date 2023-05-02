import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.node import Node

from robosub_interfaces.msg import Position, RFM9xPayload, ID
from std_msgs.msg import Float64

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
        self._coord_sub = self.create_subscription(Position, 'coordinates',
                self._coord_sub_callback, 1, callback_group=self._sub_callback_group)
        
        self._depth_sub = self.create_subscription(Float64, 'depth',
            self._depth_sub_callback, 1, callback_group=self._sub_callback_group)
        self._roll_sub = self.create_subscription(Float64, 'actualRoll',
            self._roll_sub_callback, 1, callback_group=self._sub_callback_group)
        self._pitch_sub = self.create_subscription(Float64, 'actualPitch',
            self._pitch_sub_callback, 1, callback_group=self._sub_callback_group)
        self._yaw_sub = self.create_subscription(Float64, 'actualYaw',
            self._yaw_sub_callback, callback_group=self._sub_callback_group)
        self._battery_sub = self.create_subscription(Float64, 'battery',
            self._battery_sub_callback, callback_group=self._sub_callback_group)

        # Store sequence of send messages
        self._msg_id = 0
        self.telem = TelemetryMessage()
        # Publish telemetry data for radio controller
        self._telem_pub = self.create_publisher(RFM9xPayload, 'telem_data', 1)
        # ...at intervals of 2 seconds
        self._pub_timer = self.create_timer(2.0, self._pub_timer_callback, self._timer_callback_group)

    def _resend_sub_callback(self, msg):
        self.telem.requesting_resend = True
        self.telem.resend = msg.id

    def _coord_sub_callback(self, msg):
        self.telem.position.latitude = msg.lat
        self.telem.position.longitude = msg.lon

    def _roll_sub_callback(self, msg):
        self.telem.pose.roll = msg.data
        
    def _pitch_sub_callback(self, msg):
        self.telem.pose.pitch = msg.data
        
    def _yaw_sub_callback(self, msg):
        self.telem.pose.yaw = msg.data

    def _battery_sub_callback(self, msg):
        self.telem.battery = msg.battery

    def _pub_timer_callback(self):
        self.telem.id = self._msg_id
        self._msg_id += 1

        telem_data = [byte for byte in self.telem.SerializeToString()]
        self._telem_pub.publish(RFM9xPayload(payload=telem_data))
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
