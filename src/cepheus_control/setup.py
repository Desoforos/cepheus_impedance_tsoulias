from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cepheus_control'],  # Replace with your Python package name
    package_dir={'': 'src'},
)

setup(**setup_args)
