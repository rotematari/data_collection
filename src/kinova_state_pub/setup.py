from setuptools import find_packages, setup
from os import path
from glob import glob
package_name = 'kinova_state_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/kinova_and_natnet.launch.py']),
        ('share/' + package_name + '/config', glob(path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rotem',
    maintainer_email='rotem.atri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable name = module:function
            'end_effector_pose_pub_node = kinova_state_pub.kinova_state_pub_node:main',
        ],
    },
)
