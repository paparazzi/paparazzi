OCAMLC=ocamlc -thread
OCAMLOPT=ocamlopt -thread
INCLUDES=-I +lablgtk2 -I +camlimages -I ../../lib/ocaml
LIBS=glibivy-ocaml.cma lablgtk.cma ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)


all : map2d

opt : map2d.opt


map2d : map2d.ml ../../lib/ocaml/lib-pprz.cma ../../lib/ocaml/xlib-pprz.cma
	$(OCAMLC) -custom $(INCLUDES) $(LIBS) threads.cma gtkThread.cmo gtkInit.cmo $< -o $@

map2d.opt : map2d.cmx
	$(OCAMLOPT) $(INCLUDES) str.cmxa unix.cmxa xml-light.cmxa $(LIBS:.cma=.cmxa) threads.cmxa gtkThread.cmx gtkInit.cmx $< -o $@


.SUFFIXES: .ml .mli .cmo .cmi .cmx

%.cmo: %.ml
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
%.cmi: %.ml
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
%.cmx: %.ml
	$(OCAMLOPT) $(INCLUDES) -labels -w s -c $<

clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d
