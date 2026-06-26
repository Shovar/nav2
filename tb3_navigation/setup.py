from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml') + glob('map/*.pgm') + glob('map/*.md')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@user.com',
    description='Sistema de navegación autónomo con Pure Pursuit',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'pure_pursuit_navigator = tb3_navigation.navigation_node:main',
        ],
    },
)
