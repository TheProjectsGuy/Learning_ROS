## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

"""
    Module setuptools is used for building, distributing and installing python modules.
    We need to make a function call to 'setup' in order to setup everything. It takes author
    details and package details. We have another function provided by catkin to handle generating
    that dictionary for the 'setup' function.

    Reference: https://docs.python.org/3/distutils/setupscript.html
"""
from setuptools import setup
"""
    Import a function that will read the Package.xml file of the package and give us the keys for
    the setup function. This will create the dictionary for the 'setup' function imported above.

    Doc: https://docs.ros.org/en/independent/api/catkin_pkg/catkin_pkg.html#catkin_pkg.python_setup.generate_distutils_setup
"""
from catkin_pkg.python_setup import generate_distutils_setup

# Create dictionary for setup using package.xml
setup_args = generate_distutils_setup(
    # A list of packages in the repository
    packages=[
            'simple_module', # In ./src/simple_module
            'basic_math',    # In ./src/basic_math (Note: You could include other modules)
        ],
    # Directory in which packages are located (usually `src`)
    package_dir={'': 'src'}
)

setup(**setup_args)
