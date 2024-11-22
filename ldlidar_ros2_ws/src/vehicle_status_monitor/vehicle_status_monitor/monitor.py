# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import time

# class VehicleStatusMonitor(Node):
#     def __init__(self):
#         super().__init__('vehicle_status_monitor')
#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/scan',  # Replace with your sensor's topic name
#             self.listener_callback,
#             10)
#         self.last_data_time = time.time()
#         self.sensor_status = "unknown"  # Initial status
#         self.timer = self.create_timer(1.0, self.check_sensor_status)  # Check every second

#     def listener_callback(self, msg):
#         # Update the last received time whenever data is received
#         self.get_logger().info('Data received from sensor.')
#         self.last_data_time = time.time()

#     def check_sensor_status(self):
#         current_time = time.time()
#         time_since_last_data = current_time - self.last_data_time

#         if time_since_last_data > 2.0:  # Threshold for malfunction
#             if self.sensor_status != "malfunction":  # Only log once if already malfunctioning
#                 self.sensor_status = "malfunction"
#                 self.generate_report('Sensor data missing or malfunction detected.')
#         else:  # Sensor is operational
#             if self.sensor_status != "operational":  # Only log once if already operational
#                 self.sensor_status = "operational"
#                 self.generate_report('Sensor is operational.')

#     def generate_report(self, message):
#         # Log the message and write it to a file
#         self.get_logger().info(message) if "operational" in message else self.get_logger().warn(message)
#         with open('vehicle_status_report.txt', 'a') as report_file:
#             report_file.write(f'{time.ctime()} - {message}\n')

# def main(args=None):
#     rclpy.init(args=args)
#     monitor_node = VehicleStatusMonitor()
#     try:
#         rclpy.spin(monitor_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         monitor_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time

class VehicleStatusMonitor(Node):
    def __init__(self):
        super().__init__('vehicle_status_monitor')
        
        # Subscriber to LaserScan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your sensor's topic name
            self.listener_callback,
            10)
        
        # Publisher to publish the sensor status
        self.publisher_ = self.create_publisher(String, 'vehicle_status_data', 10)
        
        self.last_data_time = time.time()
        self.sensor_status = "unknown"  # Initial status
        self.timer = self.create_timer(1.0, self.check_sensor_status)  # Check every second

    def listener_callback(self, msg):
        # Update the last received time whenever data is received
        #self.get_logger().info('Data received from sensor.')
        self.last_data_time = time.time()

    def check_sensor_status(self):
        current_time = time.time()
        time_since_last_data = current_time - self.last_data_time

        status_message = String()  # Message to publish
        if time_since_last_data > 2.0:  # Threshold for malfunction
            if self.sensor_status != "malfunction":  # Only log once if already malfunctioning
                self.sensor_status = "malfunction"
                self.generate_report('Sensor data missing or malfunction detected.')
            
            status_message.data = "Sensor status: Malfunction"
        else:  # Sensor is operational
            if self.sensor_status != "operational":  # Only log once if already operational
                self.sensor_status = "operational"
                self.generate_report('Sensor is operational.')
            
            status_message.data = "Sensor status: Operational"
        
        # Publish the status message
        self.publisher_.publish(status_message)
        self.get_logger().info(f"Published: {status_message.data}")

    def generate_report(self, message):
        # Log the message and write it to a file
        self.get_logger().info(message) if "operational" in message else self.get_logger().warn(message)
        with open('vehicle_status_report.txt', 'a') as report_file:
            report_file.write(f'{time.ctime()} - {message}\n')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = VehicleStatusMonitor()
    try:
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


