from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gar_gauge_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config', 'servo'), glob('config/servo/*.yaml')),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_write_node = gar_gauge_control.read_write_node:main'
        ],
    },
)
