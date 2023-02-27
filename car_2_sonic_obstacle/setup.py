from setuptools import setup

package_name = 'car_2_sonic_obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmakise',
    maintainer_email='kmakise@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sonic_obstacle = car_2_sonic_obstacle.sonic_obstacle:main',
        'radar_obstacle = car_2_sonic_obstacle.radar_obstacle:main',
        ],
    },
)
