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

#ifndef BOXES_CONSTANTS_H
#define BOXES_CONSTANTS_H

// Match types
#define MATCH_TYPE_NORMAL                  0
#define MATCH_TYPE_RADIUS                  1

// Feature Detector
/* Available feature detectors: FAST, STAR, ORB, BRISK, MSER, GFTT, HARRIS,
     Dense, SimpleBlob, Grid, Pyramid and (SIFT, SURF which are non-free) */

#define FEATURE_DETECTOR_FAST                     "FAST"
#define FEATURE_DETECTOR_GFTT                     "GFTT"
#define FEATURE_DETECTOR_ORB                      "ORB"
#define FEATURE_DETECTOR_PYRAMID_FAST             "PyramidFAST"
#ifdef BOXES_NONFREE
# define FEATURE_DETECTOR_SIFT                    "SIFT"
# define FEATURE_DETECTOR_SURF                    "SURF"

# define DEFAULT_FEATURE_DETECTOR                 "SURF"
#else
# define DEFAULT_FEATURE_DETECTOR                 "PyramidFAST"
#endif

/* Available feature extractors: ORB, BRISK, BRIEF
     (SIFT, SURF which are non-free) */
#define FEATURE_DETECTOR_EXTRACTOR_ORB            "ORB"
#ifdef BOXES_NONFREE
# define FEATURE_DETECTOR_EXTRACTOR_SIFT          "SIFT"
# define FEATURE_DETECTOR_EXTRACTOR_SURF          "SURF"

# define DEFAULT_FEATURE_DETECTOR_EXTRACTOR       "SIFT"
#else
# define DEFAULT_FEATURE_DETECTOR_EXTRACTOR       "ORB"
#endif

#define CAMERA_EXTENSION                          "camera"
#define NURBS_CURVE_EXTENSION                     "nurbs"

#define WHITESPACE                                " \t"

/*
 * Private definitions that may only be used internally.
 */
#ifdef BOXES_PRIVATE

#include <boxes/converters.h>

//#define FEATURE_MATCHER_REPROJECTION_ERROR
#define REPROJECTION_ERROR_MAX		200.0

#define DEFAULT_MATCH_VALID_RATIO	"0.8"
#define DEFAULT_EPIPOLAR_DISTANCE_FACTOR	"0.001"

// Optical Flow constants
#define OPTICAL_FLOW_ALGO_FARNEBACK

#define OPTICAL_FLOW_USE_GREYSCALE_IMAGES
// #define OPTICAL_FLOW_USE_GFTT
#define OF_SEARCH_WINDOW_SIZE           50
#define OF_MAX_PYRAMIDS                  5
#define OF_MAX_VERROR                    5.0
#define OF_RADIUS_MATCH                 (float)OF_SEARCH_WINDOW_SIZE

// Triangulation
#define TRIANGULATION_MAX_ITERATIONS    10
#define TRIANGULATION_EPSILON            0.001

// Point Cloud constants
#define POINT_CLOUD_TRIANGULATION_SEARCH_RADIUS            1000
#define POINT_CLOUD_TRIANGULATION_MULTIPLIER                  5.0
#define POINT_CLOUD_TRIANGULATION_MAX_NEAREST_NEIGHBOUR    1000
#define POINT_CLOUD_TRIANGULATION_MAX_SURFACE_ANGLE        DEG2RAD(30)
#define POINT_CLOUD_TRIANGULATION_MIN_ANGLE                DEG2RAD(5)
#define POINT_CLOUD_TRIANGULATION_MAX_ANGLE                DEG2RAD(180)

// SURF
#define DEFAULT_SURF_MIN_HESSIAN       "300"

#endif

#endif
