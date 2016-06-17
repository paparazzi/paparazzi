# Math library

Math lib used in all airborne code of paparazzi.

See also the @ref math docs.

HOWTO install a shared library to use in other projects
-------------------------------------------------------

1. Build library: in sw/airborne/math, type
  `make shared_lib`

  The default build directory is var/build/math,
  to change it: `BUILDDIR=<your_build_dir> make shared_lib`

2. Install library: in this folder, type
  `make install_shared_lib`

  the default install dir is /usr/local
  and will install files in
  * /usr/local/lib
  * /usr/local/lib/pkgconfig
  * /usr/local/include/pprz

  to change the install dir: `PREFIX=<your_install_dir> make install_shared_lib`

  note that the default install dir needs root privilege and is usually called via
  `sudo make shared_lib`

HOWTO use the shared library
----------------------------

1. with `pkg-config --cflags --libs pprzmath`

2. by hand:
- LIBS: `-L<prefix>/lib -lpprzmath`
- CFLAGS: `-I<prefix>/include/pprz`


"make clean" will only clean the build directory

