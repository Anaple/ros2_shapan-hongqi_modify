from setuptools import setup

package_name = 'car_4_communication'

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
        'net_work = car_4_communication.net_work:main',
        'vr_test = car_4_communication.vr_test:main',
        'point_data_bag = car_4_communication.point_data_bag:main',
        'tts_node = car_4_communication.tts_node:main'
        ],
    },
)
