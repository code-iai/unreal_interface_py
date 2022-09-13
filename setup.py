from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['unreal_interface_py'],
   package_dir={'': 'src'}
)

setup(**d)


