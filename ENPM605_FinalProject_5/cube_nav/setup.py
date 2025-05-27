from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cube_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hamssvarthan Ravichandar',
    maintainer_email='rhamsaa@umd.edu',
    description='ENPM605 Final Project Package',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_nav = cube_nav.cube_nav:main',
             'nav = cube_nav.navigator:main',
        ],
    },
)
