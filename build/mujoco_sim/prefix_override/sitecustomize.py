import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/czx/mujoco_learn/wheel_/wheelLqr_ws_v2_LQR/install/mujoco_sim'
