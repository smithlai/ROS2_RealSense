from setuptools import setup
import os
from glob import glob

package_name = 'realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))   # copy launch/*.launch.py to install/share/
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smith',
    maintainer_email='smith@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = '+package_name+'.simple_publisher:main',    # define python name
            'simple_subscriber = '+package_name+'.simple_subscriber:main',    # define python name
        ],
    },
)
