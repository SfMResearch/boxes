/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

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
