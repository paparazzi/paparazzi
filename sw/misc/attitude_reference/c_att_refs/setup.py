from distutils.core import setup, Extension
from Cython.Build import cythonize
from distutils.extension import Extension
import numpy

from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
pprz_src = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
pprz_airborne = path.join(pprz_src, "sw/airborne")

common_inc_dirs = ["./", path.join(pprz_src, "sw/include"), pprz_airborne, numpy.get_include()]

includedirs = common_inc_dirs + [path.join(pprz_airborne, "firmwares/rotorcraft")]
ext_quat_float = Extension("ref_quat_float",
                           sources=['ref_quat_float.pyx', path.join(pprz_airborne, 'math/pprz_algebra_float.c'),
                                    path.join(pprz_airborne, "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.c")],
                           include_dirs=includedirs,
                           extra_compile_args=["-std=c99", "-DSTABILIZATION_ATTITUDE_TYPE_FLOAT"])
ext_quat_int = Extension("ref_quat_int",
                         sources=['ref_quat_int.pyx',
                                  path.join(pprz_airborne, 'math/pprz_trig_int.c'),
                                  path.join(pprz_airborne, 'math/pprz_algebra_int.c'),
                                  path.join(pprz_airborne, "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.c")],
                         include_dirs=includedirs,
                         extra_compile_args=["-std=c99", "-DSTABILIZATION_ATTITUDE_TYPE_INT"])

extensions = [ext_quat_float, ext_quat_int]

setup(
    ext_modules=cythonize(extensions)
)
