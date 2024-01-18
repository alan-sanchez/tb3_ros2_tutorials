from setuptools import find_packages, setup

package_name = 'tb3_ros2_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alan G. Sanchez',
    maintainer_email='sanchala@oregonstate.edu',
    description='ROS2 Tutorials',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move        = tb3_ros2_tutorials.move:main',
            'scan_filter = tb3_ros2_tutorials.scan_filter:main',
        ],
    },
)
