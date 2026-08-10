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
        ('share/' + package_name + '/data', [f for f in glob('data/**/*', recursive=True) if os.path.isfile(f)]),
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
            'control_server = kuka_ros2_demo.control_server:main',
            'voice_ai_node = kuka_ros2_demo.voice_ai_node:main',
            'voice_terminal_mock = kuka_ros2_demo.voice_terminal_mock:main',
            'voice_bridge_node = kuka_ros2_demo.voice_bridge:main',
            'bridge_node = kuka_ros2_demo.bridge_node:main',
            'vision_node = kuka_ros2_demo.vision_node:main',
            'pick_place_coordinator = kuka_ros2_demo.pick_place_coordinator:main',
            'axis_node = kuka_ros2_demo.axis_sweep:main',
            'grid_node = kuka_ros2_demo.grid_coordinator:main',
            'detect_node = kuka_ros2_demo.vision_detect_node:main',
            'recorder = kuka_ros2_demo.recorder:main',
            'benchmark_logger = kuka_ros2_demo.benchmark_logger:main',
        ],
    },
)
