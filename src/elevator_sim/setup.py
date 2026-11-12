from glob import glob

from setuptools import setup

package_name = 'elevator_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud nassef',
    maintainer_email='mahmoud.nassef@stud.th-deg.de',
    description='Dual TurtleBot3 multi-floor elevator simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevator_controller = elevator_sim.elevator_controller:main',
            'mission = elevator_sim.mission:main',
            'generate_building = elevator_sim.building_gen:main',
            'elevator_cmd = elevator_sim.elevator_cli:main',
        ],
    },
)
