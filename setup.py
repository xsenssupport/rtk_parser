
import warnings
warnings.filterwarnings("ignore")

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rtk_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyrtcm', 'geographiclib'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='RTCM Parser for RTK positioning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtk_parser_node = rtk_parser.rtk_parser_node:main',
        ],
    },
)