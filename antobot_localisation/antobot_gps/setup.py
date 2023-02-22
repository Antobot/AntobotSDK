from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['antobot_gps_urcu'],
    package_dir={'': 'src'}
)

setup(**d)
