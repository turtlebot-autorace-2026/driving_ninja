from setuptools import find_packages, setup
import glob

package_name = 'mission_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob.glob('models/*.pt') + glob.glob('models/*.onnx')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wikiwiki1006',
    maintainer_email='wikiwiki041006@gmail.com',
    description='Mission sign detection (YOLO11n): publishes the current mission name on /mission_mode',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = mission_model.detection:main',
        ],
    },
)
