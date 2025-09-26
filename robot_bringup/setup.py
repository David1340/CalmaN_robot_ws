from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),  # <-- Adiciona os arquivos de launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu@email.com',
    description='Pacote de bringup do robô',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odom = robot_bringup.encoder_odom_node:main',
        ],
    },
)
