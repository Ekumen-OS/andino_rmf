from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'andino_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    (os.path.join('share',package_name,'launch'),glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share',package_name,'config'),glob(os.path.join('config', '*.[pxy][ymal]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='peera.tienthong@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'andino_server = andino_fleet.andino_controller_server:main',
            'fleet_manager = andino_fleet.fleet_manager_server:main'
        ],
    },
)
