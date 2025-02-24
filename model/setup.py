from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

# to build the extension, run:
# python setup.py build_ext --inplace

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path
    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the `get_include()`
    method can be invoked. """

    def __str__(self):
        import pybind11
        return pybind11.get_include()

ext_modules = [
    Extension(
        'PidControllerWrapper',
        ['PidControllerWrapper.cpp', '../src/PidController.cpp', '../src/Utility.cpp', 'MockArduino.cpp'],
        include_dirs=[
            # Path to pybind11 headers
            get_pybind_include(),
            # Path to the directory containing PidController.h and IEncoder.h
            '../src',
            '../test' # For the Arduino.h dummy header
        ],
        language='c++'
    ),
]

setup(
    name='PidControllerWrapper',
    version='0.0.1',
    author='Your Name',
    author_email='your.email@example.com',
    description='A pybind11 wrapper for the PidController class',
    ext_modules=ext_modules,
    install_requires=['pybind11>=2.5.0'],
    cmdclass={'build_ext': build_ext},
    zip_safe=False,
)
