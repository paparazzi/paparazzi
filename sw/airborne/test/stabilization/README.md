Some Cython wrappers for stabilization reference generation testing.

Tested with Cython 0.19

To build execute in this directory:

    python setup.py build_ext --inplace

Run the example script to compare float and int ref quat implementations:

    ./compare_ref_quat.py
