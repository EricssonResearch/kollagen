import setuptools
from distutils.core import Extension
from setuptools.command.build_ext import build_ext

compiler_options = {
            'unix': ["-std=c++20","-O3", "-Wall", "-shared","-fpic"],
            'msvc': ["/std:c++20","/Ox", "/Wall", "/utf-8", "/D_USE_MATH_DEFINES"],
        }

class build_ext_subclass(build_ext):
    def build_extensions(self):
        c = self.compiler.compiler_type
        if c in compiler_options:
           for e in self.extensions:
               e.extra_compile_args += compiler_options[c]
        build_ext.build_extensions(self)

kollagen_module = Extension(
    'kollagen',
    sources=['python/pykollagen/pykollagen.cpp'],
    include_dirs=['include'],
)

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="kollagen",
    version="1.0.0",
    author="Roberto C. Sundin",
    author_email="roberto.castro.sundin@ericsson.com",
    description=(""),
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/EricssonResearch/kollagen",
    packages=setuptools.find_packages(where="python"),
    package_dir={"": "python"},
    entry_points={"console_scripts": ["kollagenr8=pykollagen.pykollagen:gen"]},
    ext_modules=[kollagen_module],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
    cmdclass = {'build_ext': build_ext_subclass},
)
