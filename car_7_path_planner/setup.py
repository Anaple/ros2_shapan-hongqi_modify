import os
from glob import glob
from setuptools import setup

package_name = 'car_7_path_planner'
py_list = glob('./{}/*.py'.format(package_name))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name),py_list),
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
        'global_path_planning = car_7_path_planner.global_path_planning:main',
        'gps_local_path_planning = car_7_path_planner.gps_local_path_planning:main',
        'magnetic_local_path_planning = car_7_path_planner.magnetic_local_path_planning:main',

        ],
    },
)
