import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), ['package.xml'])
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'numpy',
        'opencv-python',
        'scipy',
        'matplotlib',
        'rosbags',
        'tqdm',
        'numba',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            "unix_timestamp = sensors.unix_timestamp:main",
            "uwb_node = sensors.uwb_node:main",
            "uwb_test_sender = sensors.uwb_test_sender:main",
        ],
    },
)