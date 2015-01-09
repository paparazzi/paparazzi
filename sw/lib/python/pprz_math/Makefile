
all:
	swig2.0 -python -keyword -I../../../airborne geodetic.i
	swig2.0 -python -keyword -I../../../airborne algebra.i
	python setup.py build_ext --inplace

clean:
	rm -rf build *.so *.c *.pyc geodetic.py algebra.py
