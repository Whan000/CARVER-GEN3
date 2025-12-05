from setuptools import find_packages
from setuptools import setup

setup(
    name='amt212ev_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('amt212ev_interfaces', 'amt212ev_interfaces.*')),
)
