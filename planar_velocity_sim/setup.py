from setuptools import find_packages, setup

package_name = 'planar_velocity_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaepoong Lee',
    maintainer_email='ske03005@cbnu.ac.kr',
    description='Minimal planar simulator driven by body-frame vx, vy, and yaw rate.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planar_velocity_sim_node = planar_velocity_sim.planar_velocity_sim_node:main',
        ],
    },
)
