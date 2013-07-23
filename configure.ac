
AC_PREREQ([2.64])

AC_INIT([boxes],
	[0.0.1])

AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_AUX_DIR([build-aux])

AM_INIT_AUTOMAKE([
	foreign
	1.11
	-Wall
	-Wno-portability
	silent-rules
	tar-pax
	no-dist-gzip
	dist-xz
	subdir-objects
])
AM_SILENT_RULES([yes])

LT_PREREQ(2.2)
LT_INIT([disable-static])

AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_CC_C_O
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11([noext], [mandatory])

AC_PATH_PROG([M4], [m4])

# This makes sure pkg.m4 is available.
m4_pattern_forbid([^_?PKG_[A-Z_]+$],[*** pkg.m4 missing, please install pkg-config])

PKG_CHECK_MODULES(OPENCV, [opencv >= 2.4.5])

AC_CONFIG_FILES([
	Makefile
])

AC_OUTPUT
AC_MSG_RESULT([
	$PACKAGE_NAME $VERSION

	CFLAGS:			${CFLAGS}
	CXXFLAGS:		${CXXFLAGS}
	CPPFLAGS:		${CPPFLAGS}
	LDFLAGS:		${LDFLAGS}
])
