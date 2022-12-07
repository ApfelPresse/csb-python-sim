import os

import numpy
from setuptools import Extension, setup, find_packages

USE_CYTHON = os.getenv("USE_CYTHON", True)

ext = '.pyx' if USE_CYTHON else '.c'

__version__ = "0.0.2"
extensions = [
    Extension("csb_sim",
              [f"src/main{ext}"],
              include_dirs=[numpy.get_include()],
              define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")]
              ),
]

if USE_CYTHON:
    from Cython.Build import cythonize

    extensions = cythonize(extensions,
                           compiler_directives={
                               'language_level': "3",
                               'cdivision': True,
                           })

setup(
    name="csb_sim",
    version=__version__,
    author="Kevin Stiefel",
    author_email="",
    url="https://github.com/ApfelPresse/csb-python-sim",
    description="Coders Strike Back - Simulation in Python, converted from codinggame.com https://github.com/inoryy/csb-ai-starter",
    long_description="",
    # setup_requires = ["cython", "numpy"],
    packages=find_packages(),
    install_requires=["numpy"],
    # include_package_data=True,
    # cmdclass = {'build_ext': build_ext},
    ext_modules=extensions,
    zip_safe=False,
    python_requires=">=3.7",
)
