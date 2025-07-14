from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'dashboard_json'), glob('dashboard_layout/*.json'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebrahim-abdelghfar',
    maintainer_email='ibrahim.abdelghafar@dakahlia.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'micro_ros_agent_launcher = robot_bringup.robot_bringup:main',
            'dashboard_launcher = robot_bringup.open_dashboard:main',
        ],
    },
)
