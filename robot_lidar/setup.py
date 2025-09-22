from setuptools import find_packages, setup

package_name = 'robot_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodri',
    maintainer_email='rodri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidarX4_node = robot_lidar.lidarX4_node:main',
            'lidarX2_node = robot_lidar.lidarX2_node:main',
        ],
    },
)
