from setuptools import find_packages, setup

package_name = 'node_package'

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
    maintainer='valeria',
    maintainer_email='valeria@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_publisher_node = node_package.command_publisher_node:main',
            'jerry_robot_node = node_package.jerry_robot_node:main',
            'client_node = node_package.client_node:main'
        ],
    },
)
