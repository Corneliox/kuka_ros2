from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'kuka_ros2_demo'
# Collect all Vosk model files recursively
model_files = []
for path in glob('kuka_ros2_demo/vosk-model-small-en-us/**/*', recursive=True):
    if os.path.isfile(path):
        install_path = os.path.join(
            'lib/python3.12/site-packages',
            os.path.dirname(path)
        )
        model_files.append((install_path, [path]))
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ] + model_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emil',
    maintainer_email='emilphilvinode3vsdcityp@gmail.com',
    description='Surgical instrument pick and place demo',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'surgical_pick_place = kuka_ros2_demo.surgical_pick_place:main',
            'multi_instrument_pick_place = kuka_ros2_demo.multi_instrument_pick_place:main',
            'surgical_control_server = kuka_ros2_demo.surgical_control_server:main',
            'voice_ai_node = kuka_ros2_demo.voice_ai_node:main',
            'voice_terminal_mock = kuka_ros2_demo.voice_terminal_mock:main',
            'voice_bridge_node = kuka_ros2_demo.voice_bridge:main',
            'bridge_node = kuka_ros2_demo.bridge_node:main',
            'vision_node = kuka_ros2_demo.vision_node:main',
            'vision_debug = kuka_ros2_demo.vision:main',
            'pick_place_coordinator = kuka_ros2_demo.pick_place_coordinator:main',
            'hover_xy_node = kuka_ros2_demo.hover_xy_node:main',
            'axis_node = kuka_ros2_demo.axis_sweep:main',
            'grid_node = kuka_ros2_demo.grid_coordinator:main',
        ],
    },
)
