import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'new_uwb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS2 reception/control for the uwb_sw (NXP board) CMD_* protocol.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            "new_uwb_udp_frame_publisher = new_uwb.new_uwb_udp_frame_publisher:main",
            "new_uwb_node = new_uwb.new_uwb_node:main",
            "new_uwb_test_sender = new_uwb.new_uwb_test_sender:main",
        ],
    },
)
