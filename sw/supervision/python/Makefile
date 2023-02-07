# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.

SOURCEDIR = ui
BUILDDIR = generated

SOURCES = $(wildcard $(SOURCEDIR)/*.ui)
OBJECTS = $(patsubst $(SOURCEDIR)/%.ui,$(BUILDDIR)/%.py,$(SOURCES))

CC = pyuic5

all: $(BUILDDIR) $(OBJECTS)


$(OBJECTS): $(BUILDDIR)/%.py : $(SOURCEDIR)/%.ui
	$(CC) -o $@ $<;

$(BUILDDIR):
	mkdir -p $@

clean:
	rm -r generated


.PHONY: build_dir
