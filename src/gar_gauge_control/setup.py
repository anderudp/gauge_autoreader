from setuptools import find_packages, setup

package_name = 'gar_gauge_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'calibrator_node = gar_gauge_control.calibrator_node:main',
            'read_write_node = gar_gauge_control.read_write_node:main'
        ],
    },
)
