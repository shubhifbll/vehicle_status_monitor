import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vijay/ldlidar_ros2_ws/install/pyside2_gui'
