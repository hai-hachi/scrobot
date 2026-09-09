from glob import glob
from setuptools import find_packages, setup

package_name = 'scrobot_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scrobot',
    maintainer_email='user@example.com',
    description='Evaluation tools for SC Robot localization, odometry, and navigation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_odom_logger = scrobot_evaluation.local_odom_logger:main',
            'local_odom_test_runner = scrobot_evaluation.local_odom_test_runner:main',
            'analyze_local_odom = scrobot_evaluation.analyze_local_odom:main',
        ],
    },
)
