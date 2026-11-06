from setuptools import find_packages, setup

package_name = 'level_crossing'

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
    maintainer='asilia',
    maintainer_email='keummingi0131@gmail.com',
    description='Level Crossing Node for turtlebot3_autorace_2025',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = level_crossing.detection:main',
        ],
    },
)
