from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['common_tello_application'],
	package_dir={'': 'src/apriltag'}
)

setup(**d)
