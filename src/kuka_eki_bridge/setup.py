from setuptools import find_packages, setup

package_name = 'kuka_eki_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
       ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/config', ['config/eki_controllers.yaml']),
       ('share/' + package_name + '/launch', ['launch/eki_moveit_planning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emil',
    maintainer_email='emilphilvinode3vsdcityp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'bridge_node = kuka_eki_bridge.bridge_node:main',
        'voice_bridge_node = kuka_eki_bridge.voice_bridge:main',
        'gripper_bridge = kuka_eki_bridge.gripper_bridge:main',
        'controller_node = kuka_eki_bridge.kuka_eki_controller_node:main',
        ],
    },
)
