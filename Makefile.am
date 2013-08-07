
CXXFLAGS_PRIVATE = -DBOXES_PRIVATE

AM_CPPFLAGS = \
	-include $(top_builddir)/config.h \
	-I $(top_srcdir)/include

ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES =
noinst_LTLIBRARIES =

bin_PROGRAMS = boxes

include_HEADERS = \
	include/boxes.h

pkginclude_HEADERS =

#- boxes ----------------------------------------------------------------------

boxes_SOURCES = \
	src/main.cc

boxes_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	$(PCL_CFLAGS)

boxes_LDADD = \
	libboxes.la

#- libboxes -------------------------------------------------------------------

lib_LTLIBRARIES += \
	libboxes.la

libboxes_la_SOURCES = \
	src/lib/camera_matrix.cc \
	src/lib/boxes.cc \
	src/lib/feature_matcher.cc \
	src/lib/image.cc \
	src/lib/point_cloud.cc

libboxes_la_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	$(CXXFLAGS_PRIVATE) \
	$(OPENMP_CFLAGS) \
	$(OPENCV_CXXCFLAGS) \
	$(PCL_CFLAGS)

libboxes_la_LIBADD = \
	$(OPENCV_LIBS) \
	$(PCL_LIBS)

pkginclude_HEADERS += \
	include/boxes/camera_matrix.h \
	include/boxes/constants.h \
	include/boxes/converters.h \
	include/boxes/boxes.h \
	include/boxes/feature_matcher.h \
	include/boxes/image.h \
	include/boxes/point_cloud.h \
	include/boxes/structs.h \
	include/boxes/suppress_warnings.h
