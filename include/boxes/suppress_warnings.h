
#ifndef BOXES_SUPPRESS_WARNINGS_H
#define BOXES_SUPPRESS_WARNINGS_H

#define INCLUDE_IGNORE_WARNINGS_BEGIN \
	_Pragma("GCC diagnostic push"); \
	_Pragma("GCC diagnostic ignored \"-Wall\""); \
	_Pragma("GCC diagnostic ignored \"-Wextra\""); \
	\
	_Pragma("GCC diagnostic ignored \"-Wattributes\""); \
	_Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\""); \
	_Pragma("GCC diagnostic ignored \"-Wfloat-equal\""); \
	_Pragma("GCC diagnostic ignored \"-Wshadow\""); \
	_Pragma("GCC diagnostic ignored \"-Wundef\""); \
	_Pragma("GCC diagnostic ignored \"-Wunknown-pragmas\""); \
	_Pragma("GCC diagnostic ignored \"-Wunused-local-typedefs\"");

#define INCLUDE_IGNORE_WARNINGS_END \
	_Pragma("GCC diagnostic pop");

#endif
