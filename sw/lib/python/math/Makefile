
all:
	swig2.0 -python pprz_geodetic.i
	python setup.py build_ext --inplace

clean:
	rm -rf build *.so *.c pprz_geodetic.py
