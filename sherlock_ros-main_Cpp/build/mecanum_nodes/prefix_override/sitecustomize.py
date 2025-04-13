import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sheitej/sherlock_ros/sherlock_ros-main_Cpp/install/mecanum_nodes'
