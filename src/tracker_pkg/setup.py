from setuptools import setup
import os
from glob import glob

package_name = 'tracker_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='atswanback@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_gps_node = tracker_pkg.fake_gps_node:main',
            'fake_ui_node = tracker_pkg.fake_ui_node:main',
            'tracker_node = tracker_pkg.tracker_node:main'
        ],
    },
)
