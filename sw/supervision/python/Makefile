# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.

SOURCEDIR = ui
BUILDDIR = generated

SOURCES = $(wildcard $(SOURCEDIR)/*.ui)
OBJECTS = $(patsubst $(SOURCEDIR)/%.ui,$(BUILDDIR)/%.py,$(SOURCES))

CC = pyuic5

all: $(BUILDDIR) $(OBJECTS) resources


$(OBJECTS): $(BUILDDIR)/%.py : $(SOURCEDIR)/%.ui
	$(CC) --import-from=generated -o $@ $<;

resources:
	pyrcc5 $(SOURCEDIR)/resources.qrc -o $(BUILDDIR)/resources_rc.py

$(BUILDDIR):
	mkdir -p $@

clean_generated:
	rm -r generated


.PHONY: build_dir clean_generated
