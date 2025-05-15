from setuptools import find_packages, setup

package_name = 'cube_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'rclpy',
    'geometry_msgs',
    'tf2_ros',
    'aruco_msgs',
    'nav2_msgs',
    'nav2_simple_commander',
  ],
    zip_safe=True,
    maintainer='hv',
    maintainer_email='manojs@umd.edu',
    description='Go cube 1',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_nav = cube_nav.cube_nav:main',
        ],
    },
)
