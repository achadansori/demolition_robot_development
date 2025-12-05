import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/achadansori/STM32CubeIDE/demolition_robot_development/brokk_demolition_sim/install/brokk_control'
