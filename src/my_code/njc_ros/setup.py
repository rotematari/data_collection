import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'njc_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rotem',
    maintainer_email='rotem.atri@gmail.com',
    description='Neural Jacobian Controller ROS 2 integration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dlo_spline_node = njc_ros.dlo_spline_node:main',
            'njc_controller_node = njc_ros.njc_controller_node:main',
        ],
    },
)
