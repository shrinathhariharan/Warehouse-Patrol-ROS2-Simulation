from setuptools import find_packages, setup

package_name = 'warehouse_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shrin',
    maintainer_email='shrin@todo.todo',
    description='Warehouse patrol robot mission scheduler.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_scheduler = warehouse_patrol.patrol_scheduler:main',
        ],
    },
)
