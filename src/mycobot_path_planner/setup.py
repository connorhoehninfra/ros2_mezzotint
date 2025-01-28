import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'mycobot_path_planner'
setup(
name=package_name,
version='0.0.0',
packages=find_packages(exclude=['test']),
data_files=[
 ('share/ament_index/resource_index/packages',
 ['resource/' + package_name]),
 ('share/' + package_name, ['package.xml']),
 (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py'))
 ],
install_requires=['setuptools'],
zip_safe=True,
maintainer='Ziad Ammar',
maintainer_email='ZiadRoboticist@outlook.com',
description='Path planning package for mycobot 630 pro robotic arm',
license='Apache-2.0',
tests_require=['pytest'],
entry_points={
'console_scripts': [
'path_planner = mycobot_path_planner.path_planner:main'
 ],
 },
)