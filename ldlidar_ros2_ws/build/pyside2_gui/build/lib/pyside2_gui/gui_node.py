import sys
from PySide2.QtWidgets import QApplication, QMainWindow, QTextEdit, QVBoxLayout, QWidget
from PySide2.QtCore import QTimer
import paho.mqtt.client as mqtt


class VehicleStatusGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the main window
        self.setWindowTitle("Vehicle Status Monitor")
        self.setGeometry(100, 100, 600, 400)

        # Create a central widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Add a QTextEdit for displaying the vehicle status
        self.status_display = QTextEdit(self)
        self.status_display.setReadOnly(True)
        self.layout.addWidget(self.status_display)

        # Set up MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.status_display.append("Connected to MQTT Broker\n")
            client.subscribe("vehicle_status_topic")
        else:
            self.status_display.append("Failed to connect to MQTT Broker\n")

    def on_message(self, client, userdata, msg):
        # Append received message to the QTextEdit
        message = msg.payload.decode()
        self.status_display.append(f"Received: {message}")

    def closeEvent(self, event):
        # Stop the MQTT client when the GUI is closed
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = VehicleStatusGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

