from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'andino_fleet_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peera',
    maintainer_email='ptientho@stevens.edu',
    description='Andino Fleet Adapter',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=andino_fleet_adapter.fleet_adapter:main'
        ],
    },
)
