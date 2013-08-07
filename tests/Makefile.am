
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

TESTS = $(BOXES_BUILT_TESTS)

BOXES_BUILT_TESTS = \
	library_version

check_PROGRAMS = \
	$(BOXES_BUILT_TESTS)

library_version_SOURCES = \
	library_version.cc
