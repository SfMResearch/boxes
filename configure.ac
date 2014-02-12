
AC_PREREQ([2.64])

AC_INIT([boxes],
	[1.0.0],
	[],
	[boxes],
	[https://github.com/SfMResearch/boxes])

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

AC_PROG_SED

AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_CC_C_O
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11([noext], [mandatory])
AC_OPENMP

AC_PATH_PROG([M4], [m4])

CC_CHECK_FLAGS_APPEND([with_cflags], [CFLAGS], [\
	-pipe \
	-Wall \
	-Wextra \
	-Wno-inline \
	-Wundef \
	"-Wformat=2 -Wformat-security -Wformat-nonliteral" \
	-Wlogical-op \
	-Wsign-compare \
	-Wpointer-arith \
	-Winit-self \
	-Wfloat-equal \
	-Wredundant-decls \
	-Wmissing-declarations \
	-Wmissing-noreturn \
	-Wendif-labels \
	-Wcast-align \
	-Wstrict-aliasing=2 \
	-Wwrite-strings \
	-Wno-long-long \
	-Wno-overlength-strings \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-Wno-unused-result \
	-Werror=overflow \
	-ffast-math \
	-fno-common \
	-fdiagnostics-show-option \
	-fdiagnostics-color \
	-fno-strict-aliasing \
	-ffunction-sections \
	-fdata-sections \
	-fstack-protector-all \
	--param=ssp-buffer-size=4])
AC_SUBST([OUR_CFLAGS], $with_cflags)
AC_SUBST([OUR_CXXFLAGS], $with_cflags)

AS_CASE([$CFLAGS], [*-O[[12345g\ ]]*],
	[CC_CHECK_FLAGS_APPEND([with_cppflags], [CPPFLAGS], [\
		-Wp,-D_FORTIFY_SOURCE=2])],
	[AC_MSG_RESULT([skipping -D_FORTIFY_SOURCE, optimization not enabled])])
AC_SUBST([OUR_CPPFLAGS], $with_cppflags)

CC_CHECK_FLAGS_APPEND([with_ldflags], [LDFLAGS], [\
	-Wl,--as-needed \
	-Wl,--no-undefined \
	-Wl,--gc-sections \
	-Wl,-z,relro \
	-Wl,-z,now])
AC_SUBST([OUR_LDFLAGS], $with_ldflags)

# This makes sure pkg.m4 is available.
m4_pattern_forbid([^_?PKG_[A-Z_]+$],[*** pkg.m4 missing, please install pkg-config])

# Boost
BOOST_REQUIRE([1.49])

BOOST_REGEX([mt])
BOOST_SYSTEM([mt])

# OpenCV
PKG_CHECK_MODULES(OPENCV, [opencv >= 2.4])

AC_LANG_PUSH([C++])
AC_ARG_ENABLE([nonfree],
	AS_HELP_STRING([--enable-nonfree], [Use nonfree features from OpenCV]),
	[],
	[AC_CHECK_HEADERS([opencv2/nonfree/nonfree.hpp], [
		enable_nonfree=yes
	], [
		enable_nonfree=no
	])]
)
AC_LANG_POP([C++])

AS_IF([test "x${enable_nonfree}" = "xyes"], [
        AC_DEFINE(BOXES_NONFREE, 1, [Define if OpenCV nonfree features are available])
])

# PCL
PKG_CHECK_MODULES(PCL_APPS, [pcl_apps >= 1.6.0], [], [true])

PKG_CHECK_MODULES(PCL_COMMON, [pcl_common >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_COMMON, [pcl_common-1.7])
])
PKG_CHECK_MODULES(PCL_FEATURES, [pcl_features >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_FEATURES, [pcl_features-1.7])
])
PKG_CHECK_MODULES(PCL_FILTERS, [pcl_filters >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_FILTERS, [pcl_filters-1.7])
])
PKG_CHECK_MODULES(PCL_IO, [pcl_io >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_IO, [pcl_io-1.7])
])
PKG_CHECK_MODULES(PCL_KDTREE, [pcl_kdtree >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_KDTREE, [pcl_kdtree-1.7])
])
PKG_CHECK_MODULES(PCL_SEARCH, [pcl_search >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_SEARCH, [pcl_search-1.7])
])
PKG_CHECK_MODULES(PCL_SURFACE, [pcl_surface >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_SURFACE, [pcl_surface-1.7])
])
PKG_CHECK_MODULES(PCL_VISUALIZATION, [pcl_visualization >= 1.6.0], [], [
	PKG_CHECK_MODULES(PCL_VISUALIZATION, [pcl_visualization-1.7])
])

PCL_CFLAGS="\
	$PCL_APPS_CFLAGS \
	$PCL_COMMON_CFLAGS \
	$PCL_FEATURES_CFLAGS \
	$PCL_FILTERS_CFLAGS \
	$PCL_IO_CFLAGS \
	$PCL_KDTREE_CFLAGS \
	$PCL_SEARCH_CFLAGS \
	$PCL_SURFACE_CFLAGS \
	$PCL_VISUALIZATION_CFLAGS \
"
PCL_LIBS="\
	$PCL_APPS_LIBS \
	$PCL_COMMON_LIBS \
	$PCL_FEATURES_LIBS \
	$PCL_FILTERS_LIBS \
	$PCL_IO_LIBS \
	$PCL_KDTREE_LIBS \
	$PCL_SEARCH_LIBS \
	$PCL_SURFACE_LIBS \
	$PCL_VISUALIZATION_LIBS \
"
AC_SUBST(PCL_CFLAGS)
AC_SUBST(PCL_LIBS)

# Workaround because PCL requires headers from VTK.
PCL_CFLAGS+="-I/usr/include/vtk -I/usr/include/vtk-5.8"
#PCL_LIBS+="-L/usr/lib64/vtk -lvtkCommon -lvtkFiltering -lvtkRendering"
PCL_LIBS+="\
	-L/usr/lib64/vtk \
	-lvtkCommonCore \
	-lvtkCommonDataModel \
	-lvtkCommonMath \
	-lvtkRenderingCore \
"

AC_CONFIG_FILES([
	Makefile
	tests/Makefile
])

AC_OUTPUT
AC_MSG_RESULT([
	$PACKAGE_NAME $VERSION

	CFLAGS:			${OUR_CFLAGS} ${CFLAGS}
	CXXFLAGS:		${OUR_CXXFLAGS} ${CXXFLAGS}
	CPPFLAGS:		${OUR_CPPFLAGS} ${CPPFLAGS}
	LDFLAGS:		${OUR_LDFLAGS} ${LDFLAGS}
])
