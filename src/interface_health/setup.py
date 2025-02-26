from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'interface_health'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'panda_msgs'), glob('msg/*.msg')),  # Include custom messages
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avinashumeshsarma',
    maintainer_email='avinashumeshsarma@gmail.com',
    description='Panda Health Monitoring ROS2 Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panda_voltage_publisher = interface_health.panda_voltage_publisher:main',
            'panda_health_publisher = interface_health.panda_health_publisher:main',
        ],
    },
)
