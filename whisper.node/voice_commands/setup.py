from setuptools import setup

package_name = 'voice_commands'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giacomo Vianello',
    maintainer_email='giacomo.vianello@gmail.com',
    description='Wrap the whisper.cpp library to provide voice commands as a ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_commands = voice_commands.stream_listener:main',
        ],
    },
)
