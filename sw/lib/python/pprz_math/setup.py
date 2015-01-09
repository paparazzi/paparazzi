#!/usr/bin/env python

"""
setup.py file for pprz math wrappers
"""

from distutils.core import setup, Extension

from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
pprz_src = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
pprz_airborne = path.join(pprz_src, "sw/airborne")

common_inc_dirs = [path.join(pprz_src, "sw/include"), pprz_airborne]

geodetic_module = Extension('_geodetic',
                            sources=['geodetic_wrap.c',
                                     path.join(pprz_airborne, 'math/pprz_geodetic_int.c'),
                                     path.join(pprz_airborne, 'math/pprz_geodetic_double.c'),
                                     path.join(pprz_airborne, 'math/pprz_geodetic_float.c')
                                 ],
                            include_dirs=common_inc_dirs)
algebra_module = Extension('_algebra',
                           sources=['algebra_wrap.c',
                                    path.join(pprz_airborne, 'math/pprz_algebra_int.c'),
                                    path.join(pprz_airborne, 'math/pprz_algebra_double.c'),
                                    path.join(pprz_airborne, 'math/pprz_algebra_float.c'),
                                    path.join(pprz_airborne, 'math/pprz_trig_int.c'),
                                ],
                           include_dirs=common_inc_dirs)

setup(name='geodetic',
      version='0.1',
      author="Felix Ruess",
      description="""Pprz math wrappers""",
      ext_modules=[geodetic_module, algebra_module],
      py_modules=["geodetic", "algebra"],
      )
