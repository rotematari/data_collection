
from setuptools import find_packages, setup
from os import path
from glob import glob
package_name = 'moveit_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(path.join('launch', '*.launch.py'))),
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
            'moveit_node = moveit_pkg.moveit_pkg:main',
            'pose_goal_node = moveit_pkg.pose_goal_node:main',
            'moveit_pose_goal_node = moveit_pkg.my_moveit_node:main',
        ],
    },
)
