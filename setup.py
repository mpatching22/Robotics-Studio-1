from setuptools import setup, find_packages
from glob import glob

package_name = 'trailblazer'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/urdf_drone', glob('urdf_drone/*')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gui_node=nodes.gui_node:main',
            'flight_control=nodes.flight_control:main',
            'climb_controller=nodes.climb_controller:main',
            'pose_relay=nodes.pose_relay:main',
            'lidar_perception_360=nodes.lidar_perception_360:main',
            'front_range=nodes.front_range:main',
            'waypoint_planner=nodes.waypoint_planner:main',
            'path_planning=nodes.path_planning:main',
            'minimal_drone_nav=nodes.minimal_drone_nav:main',
        ],
    },
)
