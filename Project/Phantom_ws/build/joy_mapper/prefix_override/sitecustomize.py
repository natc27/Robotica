import sys
if sys.prefix == 'C:\\Users\\ncely\\miniforge3\\Library\\envs\\ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = 'C:\\Users\\ncely\\OneDrive\\Documentos\\GitRobotica\\Robotica\\Project\\Phantom_ws\\install\\joy_mapper'
