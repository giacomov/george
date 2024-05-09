from setuptools import setup

package_name = 'navigation'

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
    description='Interpret the voice commands and execute them on the robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interpreter = navigation.interpreter:main',
        ],
    },
)
