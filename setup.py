import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'ros2_denso_radar'
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=["radar"],
    package_dir={'': 'src', },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    install_requires=['setuptools',
                      'pyserial',
                      'python-can',
                      'cantools'],
    author='Faraz Khan',
    author_email='farazrkhan@gmail.com',
    description='Denso Radar driver.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'radar_driver = radar.driver:main',
        ],
    },
)
