#!/usr/bin/env python

"""
setup.py file for pprz_geodetic math wrapper
"""

from distutils.core import setup, Extension

from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
pprz_src = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
pprz_airborne = path.join(pprz_src, "sw/airborne")

common_inc_dirs = ["./", path.join(pprz_src, "sw/include"), pprz_airborne]

pprz_geodetic_module = Extension('_pprz_geodetic',
                                   sources=['pprz_geodetic_wrap.c',
                                            path.join(pprz_airborne, 'math/pprz_geodetic_int.c'),
                                            path.join(pprz_airborne, 'math/pprz_geodetic_double.c'),
                                            path.join(pprz_airborne, 'math/pprz_geodetic_float.c')
                                        ],
                                   include_dirs=common_inc_dirs)

setup(name='geodetic_double',
      version='0.1',
      author="Felix Ruess",
      description="""Pprz geodetic math wrapper""",
      ext_modules=[pprz_geodetic_module],
      py_modules=["pprz_geodetic"],
      )
