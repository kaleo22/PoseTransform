import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leonard/peak_cam_docker_container/PoseTranspose/install/PoseTranspose'
