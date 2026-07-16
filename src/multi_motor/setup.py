from glob import glob
import os

from setuptools import setup

package_name = 'multi_motor_control'

setup(
    name=package_name,
    version='1.0.0',
    # The rqt plugin class is imported as
    #   multi_motor_control.my_motor_rqt_plugin.MotorRQTPlugin
    # so the Python package folder must be `multi_motor_control/`.
    packages=[package_name],
    data_files=[
        # Standard ament package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Register ourselves as an rqt Python plugin.
        # ament_index convention: the installed file name MUST
        # be the package name ("multi_motor_control"), so we put
        # the source under resource/rqt_gui_py/ to avoid colliding
        # with the package marker file.
        ('share/ament_index/resource_index/rqt_gui_py__plugin',
            ['resource/rqt_gui_py/multi_motor_control']),
        ('share/' + package_name, [
            'package.xml',
            'plugin.xml',
            'calibrate.json',
        ]),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjf',
    maintainer_email='zjf@local',
    description='Denali XCR multi-motor CiA 402 control via ros2_control + rqt.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # ROS-level degree <-> encoder-count bridge for higher layers.
            'unit_converter = '
                'multi_motor_control.unit_converter:main',
            # Allow running the widget standalone for quick tests:
            #   ros2 run multi_motor_control my_motor_rqt_plugin
            'my_motor_rqt_plugin = '
                'multi_motor_control.my_motor_rqt_plugin:main',
        ],
    },
)
