from setuptools import find_packages, setup

package_name = 'scrobot_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/patrol_mission.launch.py']),
        ('share/' + package_name + '/config', ['config/patrol_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sea',
    maintainer_email='sea@example.com',
    description='Mission management for SC Robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_manager = scrobot_mission.patrol_manager:main',
        ],
    },
)
