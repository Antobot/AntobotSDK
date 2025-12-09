from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'antobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Freer',
    maintainer_email='daniel.freer@antobot.ai',
    description='Package to manage all software',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'softwareManager = antobot_manager_software.softwareManager:main'
        ],
    },

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install the config directory (including software_config.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
)
