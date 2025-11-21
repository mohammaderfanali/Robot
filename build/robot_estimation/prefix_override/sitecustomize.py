import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mohammadreza/Robotic_HW_1_3/Robotic/install/robot_estimation'
