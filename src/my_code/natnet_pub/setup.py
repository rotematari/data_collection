import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'natnet_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"),
         glob(os.path.join( 'config', "*.yaml")))

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
            'natnet_client_pub_node = natnet_pub.natnet_pub_node:main',
        ],
    },
)
