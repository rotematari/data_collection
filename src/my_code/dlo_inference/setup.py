from glob import glob
from os import path

from setuptools import find_packages, setup


package_name = "dlo_inference"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob(path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rotem",
    maintainer_email="rotem.atri@gmail.com",
    description="Real-time DLO inference package.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dlo_inference_node = dlo_inference.dlo_inference_node:main",
        ],
    },
)
