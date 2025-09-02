from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'asv_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'stable_baselines3', 'gymnasium', 'numpy>=1.22.4,<2.0', 'opencv-python'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ASV AI package for controlling autonomous surface vehicles',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asv_env_node = asv_ai.asv_env.asv_env_node:main',
            'asv_agent_node = asv_ai.asv_agent.asv_agent_node:main',
            'asv_ppo_node = asv_ai.asv_ppo.asv_ppo_node:main',
        ],
    },
)
