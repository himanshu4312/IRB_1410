from setuptools import find_packages, setup

package_name = 'irb1410_bridge'

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
    maintainer='him',
    maintainer_email='him@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = irb1410_bridge.bridge_node:main',
            'irb1410_socket_player = irb1410_bridge.irb1410_socket_player:main',

        ],
    },
)
