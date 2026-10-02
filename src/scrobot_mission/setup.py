from setuptools import find_packages, setup

package_name = 'scrobot_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sea',
    maintainer_email='sea@example.com',
    description='Four-pass sweep and local-collection mission management for SC Robot.',
    license='Apache-2.0',
)
