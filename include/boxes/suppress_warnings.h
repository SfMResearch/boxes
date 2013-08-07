
#ifndef BOXES_SUPPRESS_WARNINGS_H
#define BOXES_SUPPRESS_WARNINGS_H

#define INCLUDE_IGNORE_WARNINGS_BEGIN \
	_Pragma("GCC diagnostic push"); \
	_Pragma("GCC diagnostic ignored \"-Wall\""); \
	_Pragma("GCC diagnostic ignored \"-Wextra\""); \
	\
	_Pragma("GCC diagnostic ignored \"-Wattributes\""); \
	_Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"");

#define INCLUDE_IGNORE_WARNINGS_END \
	_Pragma("GCC diagnostic pop");

#endif
