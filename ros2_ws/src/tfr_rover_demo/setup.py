from setuptools import setup

package_name = 'tfr_rover_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TFR',
    maintainer_email='you@example.com',
    description='Publishes /cmd_vel to test Gazebo bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'circle_driver = tfr_rover_demo.circle_driver:main',
        ],
    },
)