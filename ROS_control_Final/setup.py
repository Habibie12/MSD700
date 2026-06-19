from setuptools import find_packages, setup

package_name = 'msd700'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Habibie',
    maintainer_email='user@example.com',
    description='ROS 2 serial bridge and move-distance test nodes for MSD700.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = msd700.serial_bridge:main',
            'move_distance = msd700.move_distance:main',
            'move_distance_ticks = msd700.move_distance_ticks:main',
        ],
    },
)
