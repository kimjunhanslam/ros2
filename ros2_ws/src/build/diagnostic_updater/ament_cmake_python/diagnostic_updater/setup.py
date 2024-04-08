from setuptools import find_packages
from setuptools import setup

setup(
    name='diagnostic_updater',
    version='3.1.2',
    packages=find_packages(
        include=('diagnostic_updater', 'diagnostic_updater.*')),
)
