
ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory
AUTOMAKE_OPTIONS = color-tests parallel-tests

SUBDIRS = . tests

# remove targets if the command fails
.DELETE_ON_ERROR:

# keep intermediate files
.SECONDARY:

AM_CPPFLAGS = \
	-include $(top_builddir)/config.h \
	-I $(top_srcdir)/include \
	-I $(top_srcdir)/include/third-party/moges \
	$(OUR_CPPFLAGS)

AM_CFLAGS = $(OUR_CFLAGS)
AM_CXXFLAGS = $(OUR_CXXFLAGS)
AM_LDFLAGS = $(OUR_LDFLAGS)

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
	libboxes.la \
	$(OPENCV_LIBS)

#- libboxes -------------------------------------------------------------------

lib_LTLIBRARIES += \
	libboxes.la

libboxes_la_SOURCES = \
	src/lib/camera_matrix.cc \
	src/lib/boxes.cc \
	src/lib/feature_matcher.cc \
	src/lib/feature_matcher_optical_flow.cc \
	src/lib/image.cc \
	src/lib/point_cloud.cc

libboxes_la_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	-DBOXES_PRIVATE \
	$(OPENMP_CFLAGS) \
	$(OPENCV_CXXCFLAGS) \
	$(PCL_CFLAGS)

libboxes_la_LIBADD = \
	libmoges.la \
	$(BOOST_REGEX_LIBS) \
	$(BOOST_SYSTEM_LIBS)\
	$(OPENCV_LIBS) \
	$(PCL_LIBS)

pkginclude_HEADERS += \
	include/boxes/camera_matrix.h \
	include/boxes/constants.h \
	include/boxes/converters.h \
	include/boxes/boxes.h \
	include/boxes/feature_matcher.h \
	include/boxes/feature_matcher_optical_flow.h \
	include/boxes/image.h \
	include/boxes/point_cloud.h \
	include/boxes/structs.h \
	include/boxes/suppress_warnings.h

#- libnurbs - (third-party) ---------------------------------------------------

lib_LTLIBRARIES += \
	libmoges.la

libmoges_la_SOURCES = \
	include/third-party/moges/NURBS/Curve.h \
	include/third-party/moges/NURBS/Factories.h \
	include/third-party/moges/missing.h \
	include/third-party/moges/Types.h \
	\
	src/third-party/moges/NURBS/Curve.cpp \
	src/third-party/moges/NURBS/Factories.cpp \
	src/third-party/moges/missing.cpp

libmoges_la_CXXFLAGS = \
	$(AM_CXXFLAGS)
