#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([gstreamer-camera-interface], [0.0.1])
AC_CONFIG_SRCDIR([src/CameraGstreamer.cpp])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE([foreign])
LT_INIT([disable-static])

AC_ARG_WITH(
    [camera-interface],
    AC_HELP_STRING(
        [--with-camera-interface=DIR],
        [Directory where camera-interface.h can be found]
    ),
    [CPPFLAGS="$CPPFLAGS -I$withval"]
)

PKG_CHECK_MODULES([GS], [gstreamer-1.0])
PKG_CHECK_MODULES([GSBASE], [gstreamer-base-1.0])
PKG_CHECK_MODULES([GSVIDEO], [gstreamer-video-1.0])
PKG_CHECK_MODULES([GSAPP], [gstreamer-app-1.0])

# Checks for programs.
AC_PROG_CXX
AC_LANG(C++)

# Checks for libraries.
# Checks for header files.
AC_CHECK_HEADER(
    [camera-interface.h],
    [],
    [AC_ERROR([Could not find camera-interface.h. Try specifying location using --with-camera-interface=path])]
)

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_CONFIG_FILES([Makefile
                 src/Makefile])
AC_OUTPUT
