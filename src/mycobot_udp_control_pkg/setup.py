from setuptools import find_packages, setup

package_name = 'mycobot_udp_control'

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
    maintainer='Jubin Kang',
    maintainer_email='jubineduros@gmail.com',
    description='Pick and Place, ArUco Marker UDP Controller',
    license='Team Project-T1',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_udp_launcher = mycobot_udp_control.aruco_udp_launcher:main',
            'udp_pick_place_node = mycobot_udp_control.udp_pick_place_controller:main',
        ],
    },
)
