/* File : geodetic.i */
%module geodetic
%feature("autodoc", "3");
%include "typemaps.i"
%include pprz_geodetic_int.i
%include pprz_geodetic_float.i
%include pprz_geodetic_double.i
%{
#define SWIG_FILE_WITH_INIT
%}
