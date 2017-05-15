from distutils.core import setup, Extension

module1 = Extension('giscup15',
                    sources = ['giscup15.cpp'],
                    extra_compile_args=['-std=c++11'],
                    libraries=['shp'])

setup (name = 'giscup15',
       version = '1.0',
       description = 'This is a wrapper around the shortest path engine from GIS Cup 2015 (Dijkstra, A*, ALT supported)',
       ext_modules = [module1])
