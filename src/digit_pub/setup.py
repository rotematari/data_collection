from setuptools import find_packages, setup
import os
import glob
package_name = 'digit_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets/ref_frames', glob.glob('assets/ref_frames/*.png')),
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
            'digit_pub_node = digit_pub.digit_pub_node:main',
        ],
    },
)
