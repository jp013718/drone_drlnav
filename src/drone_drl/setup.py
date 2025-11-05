from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_drl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='jp013718@ohio.edu',
    description='DRL for UAV using Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'environment = drone_drl.drl_env.env:main',
          'gazebo = drone_drl.drl_gazebo.launch_gazebo:main'
        ],
    },
)
