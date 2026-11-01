from glob import glob
from setuptools import find_packages, setup

setup(
    name='ur5e_block_bringup', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur5e_block_bringup']),
        ('share/ur5e_block_bringup', ['package.xml', 'README.md']),
        ('share/ur5e_block_bringup/launch', glob('launch/*.launch.py')),
        ('share/ur5e_block_bringup/docs', glob('docs/*.md')),
        ('share/ur5e_block_bringup/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ksn', maintainer_email='ksn@example.com', license='BSD-3-Clause',
    description='Existing UR5e/AG95 drivers with the user block model.',
    entry_points={'console_scripts': ['air_task = ur5e_block_bringup.air_task:main',
                                    'physical_stack = ur5e_block_bringup.physical_stack:main',
                                    'physical_recovery = ur5e_block_bringup.physical_recovery:main',
                                    'gripper_calibration_gui = ur5e_block_bringup.gripper_calibration_gui:main',
                                    'gripper_calibration_node = ur5e_block_bringup.gripper_calibration_node:main']},
)
