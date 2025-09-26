from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'corobot2_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gjlee',
    maintainer_email='l34004664@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wakeup_word_node=corobot2_project.wakeup_word_node:main",
            "paint_command_server=corobot2_project.paint_command_server:main",
            "paint_picker_node=corobot2_project.paint_picker_node:main",
            "color_matcher_node=corobot2_project.color_matcher_node:main",      # Ui

            # "=corobot2_project.:main",
            # "=corobot2_project.:main",

        ],
    },
)
