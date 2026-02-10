import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'traffic_sign_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'numpy<2',
        'opencv-python<4.12',
    ],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='henric.schulte@stud.tu-darmstadt.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'detect_traffic_sign_node = traffic_sign_yolo.detect_traffic_sign_node:main',
    ],
    },
)
