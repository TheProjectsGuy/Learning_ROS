## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# Reference: https://wiki.ros.org/rospy_tutorials/Tutorials/Makefile#Installing_scripts_and_exporting_modules
# Catkin docs: https://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from Package.xml
setup_args = generate_distutils_setup(
    packages=['shared_pylib'],
    package_dir={'': 'py_modules'},
)

# Setup
setup(**setup_args)
