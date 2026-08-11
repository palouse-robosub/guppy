import os, sys, subprocess
from glob import glob
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from pathlib import Path

package_name = 'guppy_teleop'

# builds qt assets
class build_qt(build_py):
    def run(self):
        qrc_src = Path("guppy_teleop/frontend/assets.qrc")
        asset_dest = Path(self.build_lib) / "guppy_teleop/frontend/rc_assets.py"

        print(f"Compiling Qt resources... {qrc_src} -> {asset_dest}")

        asset_dest.parent.mkdir(parents=True, exist_ok=True)

        #result = subprocess.run(["pyside6-rcc", str(qrc_src), "-o", str(asset_dest)])
        result = subprocess.run(["rcc", "-g", "python", str(qrc_src), "-o", str(asset_dest)])

        if (x := result.returncode) != 0:
            sys.exit(x)

        super().run()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={
        'guppy_teleop.frontend' : ['assets.qrc'],
        'guppy_teleop.frontend.ui' : ['Main.qml', 'qmldir', 'Theme.qml'],
        'guppy_teleop.frontend.ui.canvas' : ['*.qml'],
        'guppy_teleop.frontend.ui.sidebar' : ['*.qml'],
        'guppy_teleop.frontend.ui.widgets' : ['*.qml'],
        'guppy_teleop.frontend.ui.widgets.state' : ['*.qml'],
        'guppy_teleop.frontend.ui.widgets.parameter' : ['*.qml'],
        'guppy_teleop.frontend.ui.widgets.input' : ['*.qml'],
        'guppy_teleop.frontend.ui.toastify' : ['*.qml'],
        'guppy_teleop.frontend.workspaces' : ['*.json'],
        'guppy_teleop.frontend.icons' : ['*.svg'],
        'guppy_teleop.frontend.icons.toastify' : ['*.svg'],
        'guppy_teleop.frontend.workspaces' : ['*.json']
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'evdev', 'PySide6', 'pygame'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    cmdclass={'build_py': build_qt},
    entry_points={
        'console_scripts': [
            'input = guppy_teleop.input_handler:main',
            'controller = guppy_teleop.input_handler:controller',
            'keyboard = guppy_teleop.input_handler:keyboard',
            'terminal = guppy_teleop.frontend.terminal:main',
            'ksed = guppy_teleop.ksed_controller:main',

            'trans = guppy_teleop.translator:main',
            'raw = guppy_teleop.raw_controller:main',
        ],
    },
)
