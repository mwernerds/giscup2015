# Python Extension for GIS Cup 2015 algorithms

This is a minimalistic wrapper allowing to use some of the functionality from within python.

# Prerequisites

## Modern Boost Libraries

You will need an up-to-date boost distribution, 1.57.0 is known to work, but today, 1.64.0 is
current. You can install it from source. You can also just download and unpack it and modify the
setup.py file pointing the compiler to the headers. We don't need the compiled parts of boost.

However, a system-wide installation of a new boost works as follows:

     cd /usr/src
     wget https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.bz2
     tar -xaf boost_1_64_0.tar.bz2
     cd boost_1_64_0
     # It is advisable to remove all boost libraries with your package manager...
     bash ./bootstrap.sh
     ./b2
     ./b2 install # only if you want - otherwise, you need to setup paths...

## SHP Shapefile library

We use the libSHP library for reading shapefiles, which can be installed from Debian / Ubuntu using the
libshp-dev package.

# Compile and Install

     python setup.py build
     sudo python setup.py install
     python sample.py
     
# Notes for cluster deployment
You need to run python setup.py on each node in the cluster such that import giscup2015 works in python. Additionally, you will need to install libshp-dev (or at least libshp) to have the shared object file on this system. Ohterwise,
the import statement will fail, most probabily saying unknown symbol SHP_Open...
