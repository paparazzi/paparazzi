OCAMLC=ocamlc
OCAMLOPT=ocamlopt
INCLUDES=-I +lablgtk2 -I +camlimages -I ../../lib/ocaml
LIBS=glibivy-ocaml.cma lablgtk.cma ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)


all : map2d


map2d : map2d.ml
	$(OCAMLC) -custom $(INCLUDES) $(LIBS) gtkInit.cmo $< -o /dev/null #to Check
	cat ../../../pprz_src_test.sh > $@
	echo 'exec lablgtk2 -I +camlimages ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma -I $$PAPARAZZI_SRC/sw/lib/ocaml lib-pprz.cma xlib-pprz.cma $$PAPARAZZI_SRC/sw/ground_segment/cockpit/$< $$*' >> $@
	chmod a+x $@


.SUFFIXES: .ml .mli .cmo .cmi .cmx

.ml.cmo:
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
.mli.cmi:
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
.ml.cmx:
	$(OCAMLOPT) $(INCLUDES) -labels -w s -c $<

clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d
