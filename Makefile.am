#  This file is part of the boxes library.
#
#  boxes is free software; you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation; either version 2.1 of the License, or
#  (at your option) any later version.

ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory
AUTOMAKE_OPTIONS = color-tests parallel-tests

SUBDIRS = . tests

# remove targets if the command fails
.DELETE_ON_ERROR:

# keep intermediate files
.SECONDARY:

LIBBOXES_CURRENT=1
LIBBOXES_REVISION=0
LIBBOXES_AGE=0

pkgconfiglibdir=$(libdir)/pkgconfig
CLEANFILES =
EXTRA_DIST =

AM_CPPFLAGS = \
	-include $(top_builddir)/config.h \
	-I $(top_srcdir)/include \
	-I $(top_srcdir)/include/third-party \
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

dist_doc_DATA = \
	README \
	COPYING-GPL \
	COPYING-LGPL

#- boxes ----------------------------------------------------------------------

boxes_SOURCES = \
	src/main.cc

boxes_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	$(PCL_CFLAGS)

boxes_LDADD = \
	libboxes.la \
	$(OPENCV_LIBS) \
	$(PCL_LIBS)

#- libboxes -------------------------------------------------------------------

lib_LTLIBRARIES += \
	libboxes.la

libboxes_la_SOURCES = \
	src/lib/camera_matrix.cc \
	src/lib/cloud_point.cc \
	src/lib/config.cc \
	src/lib/boxes.cc \
	src/lib/feature_matcher.cc \
	src/lib/feature_matcher_optical_flow.cc \
	src/lib/image.cc \
	src/lib/multi_camera.cc \
	src/lib/point_cloud.cc \
	src/lib/util.cc \
	\
	src/third-party/moges/NURBS/Curve.cpp \
	src/third-party/moges/NURBS/Factories.cpp \
	src/third-party/moges/missing.cpp \
	include/third-party/moges/NURBS/Curve.h \
	include/third-party/moges/NURBS/Factories.h \
	include/third-party/moges/missing.h \
	include/third-party/moges/Types.h

libboxes_la_CXXFLAGS = \
	$(AM_CXXFLAGS) \
	-DBOXES_PRIVATE \
	$(OPENMP_CFLAGS) \
	$(OPENCV_CXXCFLAGS) \
	$(PCL_CFLAGS)

libboxes_la_LDFLAGS = \
	$(AM_LDFLAGS) \
	-version-info $(LIBBOXES_CURRENT):$(LIBBOXES_REVISION):$(LIBBOXES_AGE)

libboxes_la_LIBADD = \
	$(BOOST_REGEX_LIBS) \
	$(BOOST_SYSTEM_LIBS)\
	$(OPENCV_LIBS) \
	$(PCL_LIBS)

pkginclude_HEADERS += \
	include/boxes/camera_matrix.h \
	include/boxes/cloud_point.h \
	include/boxes/config.h \
	include/boxes/constants.h \
	include/boxes/converters.h \
	include/boxes/boxes.h \
	include/boxes/feature_matcher.h \
	include/boxes/feature_matcher_optical_flow.h \
	include/boxes/forward_declarations.h \
	include/boxes/image.h \
	include/boxes/multi_camera.h \
	include/boxes/point_cloud.h \
	include/boxes/structs.h \
	include/boxes/suppress_warnings.h \
	include/boxes/util.h

pkgconfiglib_DATA = \
	src/lib/boxes.pc

CLEANFILES += \
	src/lib/boxes.pc

EXTRA_DIST += \
	src/lib/boxes.pc.in

#-------------------------------------------------------------------------------

substitutions = \
	'|PACKAGE_NAME=$(PACKAGE_NAME)|' \
	'|PACKAGE_VERSION=$(PACKAGE_VERSION)|' \
	'|PACKAGE_URL=$(PACKAGE_URL)|' \
	'|prefix=$(prefix)|' \
	'|exec_prefix=$(exec_prefix)|' \
	'|libdir=$(libdir)|' \
	'|includedir=$(includedir)|'

SED_PROCESS = \
	$(AM_V_GEN)$(MKDIR_P) $(dir $@) && \
	$(SED) $(subst '|,-e 's|@,$(subst =,\@|,$(subst |',|g',$(substitutions)))) < $< > $@

%.pc: %.pc.in Makefile
	$(SED_PROCESS)
