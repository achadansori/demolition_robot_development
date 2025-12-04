from setuptools import setup
import os
from glob import glob

package_name = 'brokk_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='demolition_robot',
    maintainer_email='user@example.com',
    description='Control package for Brokk demolition robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = brokk_control.serial_bridge:main',
            'joint_controller = brokk_control.joint_controller:main',
        ],
    },
)
