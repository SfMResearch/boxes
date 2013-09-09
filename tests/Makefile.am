
ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory
AUTOMAKE_OPTIONS = color-tests parallel-tests

# remove targets if the command fails
.DELETE_ON_ERROR:

# keep intermediate files
.SECONDARY:

AM_CPPFLAGS = \
	-I $(top_srcdir)/include \
	$(OUR_CPPFLAGS)

AM_CXXFLAGS = $(OUR_CXXFLAGS) $(PCL_CFLAGS)
AM_LDFLAGS = $(OUR_LDFLAGS)

LDADD = \
	../libboxes.la

#- testsuite ------------------------------------------------------------------

BOXES_BUILT_TESTS =

TESTS = \
	$(BOXES_BUILT_TESTS)

check_PROGRAMS = \
	$(BOXES_BUILT_TESTS)


# library version

BOXES_BUILT_TESTS += library_version

library_version_SOURCES = \
	library_version.cc 


# triangulation test

BOXES_BUILT_TESTS += triangulation_test

triangulation_test_SOURCES= \
	triangulation_test.cc

triangulation_test_LDADD = \
	$(OPENCV_LIBS)
