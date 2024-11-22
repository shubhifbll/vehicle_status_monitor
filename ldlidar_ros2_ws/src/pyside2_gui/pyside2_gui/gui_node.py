import sys
from PySide2.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QTextEdit,
    QPushButton,
    QLabel,
    QHBoxLayout,
)
from PySide2.QtCore import QTimer
import paho.mqtt.client as mqtt


class MQTTGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # Timer for real-time updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_logs)
        self.timer.start(500)  # Update every 500ms

        self.logs = []

    def init_ui(self):
        self.setWindowTitle("Vehicle Status Monitor")
        self.setGeometry(200, 200, 700, 500)

        # Main layout
        main_layout = QVBoxLayout()

        # Header
        header = QLabel("Vehicle Status")
        header.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px; color : blue;")
        main_layout.addWidget(header)

        # Log viewer
        self.log_viewer = QTextEdit()
        self.log_viewer.setReadOnly(True)
        self.log_viewer.setStyleSheet("font-size: 14px; padding: 5px;")
        main_layout.addWidget(self.log_viewer)

        # Buttons layout
        button_layout = QHBoxLayout()

        self.clear_button = QPushButton("Clear Logs")
        self.clear_button.clicked.connect(self.clear_logs)
        self.clear_button.setStyleSheet("padding: 5px; font-size: 14px;")
        button_layout.addWidget(self.clear_button)
        

        self.exit_button = QPushButton("Exit")
        self.exit_button.clicked.connect(self.close_application)
        button_layout.addWidget(self.exit_button)

        main_layout.addLayout(button_layout)

        # Central widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def on_connect(self, client, userdata, flags, rc):
        self.logs.append(f"Connected to MQTT Broker with result code {rc}")
        client.subscribe("vehicle_status_topic")

    def on_message(self, client, userdata, msg):
        message = f"Received: {msg.payload.decode()}"
        self.logs.append(message)

    def update_logs(self):
        if self.logs:
            self.log_viewer.append("\n".join(self.logs))
            self.logs.clear()

    def clear_logs(self):
        self.log_viewer.clear()

    def close_application(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.close()


def main():
    app = QApplication(sys.argv)
    gui = MQTTGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
