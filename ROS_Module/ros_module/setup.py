from setuptools import find_packages, setup

package_name = 'ros_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros_module.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 module for LLM robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_input_node = ros_module.nodes.ui_input_node:main',
            'llm_receiver_node = ros_module.nodes.llm_receiver_node:main',
            'simulator_node = ros_module.nodes.simulator_node:main',
        ],
    },
)
