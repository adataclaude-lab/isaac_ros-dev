from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isaac_moveit_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='welly_liou',
    maintainer_email='Welly_Liou@adata.com',
    description='Bridge for MoveIt to Isaac Sim without Action Server',
    license='MIT', # Changed to MIT, as it's a common open-source license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_and_publish = isaac_moveit_bridge.plan_and_publish:main',
            'trajectory_executor = isaac_moveit_bridge.trajectory_executor:main',
            'keyboard_tester = isaac_moveit_bridge.keyboard_joint_tester:main'
        ],
    },
)
