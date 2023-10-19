from setuptools import setup
from glob import glob
import os

package_name = 'copter_model_identification'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('scripts/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kostya',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "logger = copter_model_identification.main:main",
        ],
    },
)
