"""README 인터페이스 변환 + robot_state_publisher.

  ros2 launch piper_host host.launch.py

드라이버 · 카메라는 띄우지 않는다 (piper-sensors / piper-record / piper-teleop 이 켜고, 이 launch 도 같이 켠다).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 실험실 로봇은 PIPER-L (표준 PIPER 와 링크 길이 · 관절 범위가 다름). 공식 agx_arm_urdf(680fa92, 팀 Dockerfile 과 같은 커밋)를
    # ~/piper_test/tools/build_piper_l_urdf.py 로 합친 파일. 관절 이름 joint1~8 은 /joint_states 와 같다.
    with open(os.path.expanduser('~/piper_test/share/piper_l/piper_l_with_gripper.urdf')) as f:
        urdf = f.read()
    config = os.path.join(get_package_share_directory('piper_host'), 'config', 'interface.yaml')
    return LaunchDescription([
        Node(package='piper_host', executable='readme_bridge', name='readme_bridge', parameters=[config], output='screen'),
        # /joint_states → base_link … link8 tf, /robot_description (transient local)
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='piper_host_rsp',
             parameters=[{'robot_description': urdf}], output='screen'),
    ])
