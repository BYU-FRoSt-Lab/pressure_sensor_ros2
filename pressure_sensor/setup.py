from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pressure_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frostlab',
    maintainer_email='frostlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_pub = pressure_sensor.pressure_pub:main',
            'pressure_to_depth = pressure_sensor.pressure_to_depth:main',
        ],
    },
)
