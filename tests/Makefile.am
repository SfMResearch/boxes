
ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}
AM_MAKEFLAGS = --no-print-directory
AUTOMAKE_OPTIONS = color-tests parallel-tests

# remove targets if the command fails
.DELETE_ON_ERROR:

# keep intermediate files
.SECONDARY:

AM_CPPFLAGS = \
	-I $(top_srcdir)/include \
	-I $(top_srcdir)/include/third-party \
	-I $(top_srcdir)/include/third-party/moges \
	$(OUR_CPPFLAGS)

AM_CXXFLAGS = $(OUR_CXXFLAGS) $(PCL_CFLAGS)
AM_LDFLAGS = $(OUR_LDFLAGS)

LDADD = \
	../libboxes.la

#- testsuite ------------------------------------------------------------------

TESTS_ENVIRONMENT = \
	TEST_DIR=$(srcdir)

BOXES_BUILT_TESTS =

TESTS = \
	$(BOXES_BUILT_TESTS)

check_PROGRAMS = \
	$(BOXES_BUILT_TESTS)

EXTRA_DIST = \
	tests.h

# Pregenerated images for testing
EXTRA_DIST += \
	images/test-square-1.jpg \
	images/test-square-2.jpg


# library version

BOXES_BUILT_TESTS += library_version

library_version_SOURCES = \
	library_version.cc 


# image load

BOXES_BUILT_TESTS += image_load

image_load_SOURCES = \
	image_load.cc


# image get

BOXES_BUILT_TESTS += image_get

image_get_SOURCES = \
	image_get.cc


## triangulation test
#
#BOXES_BUILT_TESTS += triangulation_test
#
#triangulation_test_SOURCES= \
#	triangulation_test.cc
#
#triangulation_test_LDADD = \
#	$(OPENCV_LIBS)
