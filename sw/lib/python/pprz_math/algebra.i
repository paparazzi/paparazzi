/* File : algebra.i */
%module algebra
%feature("autodoc", "3");
%include "typemaps.i"
%include pprz_algebra_int.i
%include pprz_algebra_float.i
%include pprz_algebra_double.i
%{
#define SWIG_FILE_WITH_INIT
%}
