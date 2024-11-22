import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # ROS Subscriber and Publisher
        self.subscription = self.create_subscription(
            String,
            'vehicle_status_data',  # Topic from VehicleStatusMonitor
            self.ros_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'mqtt_to_ros', 10)

        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect('localhost', 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'Connected to MQTT Broker with result code {rc}')
        client.subscribe('vehicle_status_topic')

    def on_message(self, client, userdata, msg):
        self.get_logger().info("Received from MQTT: {msg.payload.decode()}")
        ros_msg = String()
        ros_msg.data = msg.payload.decode()
        self.publisher.publish(ros_msg)

    def ros_callback(self, msg):
        self.get_logger().info(f'Sending to MQTT: {msg.data}')
        self.mqtt_client.publish('vehicle_status_topic', msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

