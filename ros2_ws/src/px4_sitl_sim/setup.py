from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'px4_sitl_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.viz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'launch', 'duke_traj'), glob('launch/duke_traj/*.csv')),
        # Avoid glob('launch/*'): it matches __pycache__ and breaks install (dirs are not files).
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='Patrick',
    maintainer_email='patrick.lo@duke.edu',
    description='PX4 SITL SIM',
    license='MIT',
    entry_points={
        'console_scripts': [
            'x500_v2_kf = px4_sitl_sim.x500_v2_kf:main',            
        ],
    },
)
