import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/achadansori/STM32CubeIDE/demolition_robot_development/install/brokk_control'
