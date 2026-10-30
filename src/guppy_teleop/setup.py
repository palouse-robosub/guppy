import os
from glob import glob

from setuptools import find_packages, setup

package_name = "guppy_teleop"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools", "evdev", "pygame"],
    zip_safe=True,
    maintainer="robosub",
    maintainer_email="robosub@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "input = guppy_teleop.input_handler:main",
            "controller = guppy_teleop.input_handler:controller",
            "keyboard = guppy_teleop.input_handler:keyboard",
            "ksed = guppy_teleop.ksed_controller:main",
            "trans = guppy_teleop.translator:main",
            "raw = guppy_teleop.raw_controller:main",
        ],
    },
)
