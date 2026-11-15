from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rf_delivery_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rf_delivery_bot']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='delivery_bot_team',
    maintainer_email='bot@example.com',
    description='Mapless indoor cafe delivery robot using RSSI gradient tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rssi_navigator = rf_delivery_bot.rssi_navigator:main',
            'rssi_filter = rssi_filter.rssi_filter_node:main',
        ],
    },
)
