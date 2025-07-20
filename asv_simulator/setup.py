from setuptools import find_packages, setup

package_name = 'asv_simulator'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    include_package_data = True,
    maintainer='Federico Peralta',
    maintainer_email='fdperalta@uloyola.es',
    description='Simulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = asv_simulator.SimulatorWrapper:main',
        ],
    },
)
