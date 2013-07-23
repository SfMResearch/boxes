
AM_CPPFLAGS = \
	-include $(top_builddir)/config.h \
	-I $(top_srcdir)/include

lib_LTLIBRARIES =
noinst_LTLIBRARIES =

bin_PROGRAMS = boxes

include_HEADERS = \
	include/boxes.h

pkginclude_HEADERS =

#- boxes ----------------------------------------------------------------------

boxes_SOURCES = \
	src/main.cc

boxes_LDADD = \
	libboxes.la

#- libboxes -------------------------------------------------------------------

lib_LTLIBRARIES += \
	libboxes.la

libboxes_la_SOURCES = \
	src/lib/boxes.cc

libboxes_la_CXXFLAGS = \
	$(OPENCV_CXXCFLAGS)

libboxes_la_LIBADD = \
	$(OPENCV_LIBS)

pkginclude_HEADERS += \
	include/boxes/boxes.h
