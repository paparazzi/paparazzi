doc_pprz_algebra.pdf: headfile.tex
	pdflatex $<
bib:
	bibtex headfile

clean:
	rm -f *~ *.aux *.bbl *.blg *.log *.out *.toc *.dvi *.ps
	find . -name '*~' -exec rm -f {} \;
