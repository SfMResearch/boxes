
#ifndef BOXES_CONSTANTS_H
#define BOXES_CONSTANTS_H

// Match types
#define MATCH_TYPE_NORMAL                  0
#define MATCH_TYPE_RADIUS                  1

/*
 * Private definitions that may only be used internally.
 */
#ifdef BOXES_PRIVATE

#include <boxes/converters.h>

#define REPROJECTION_ERROR_MAX         200.0

#define MATCH_VALID_RATIO                0.8

// Feature Detector
/* Available feature detectors: FAST, STAR, ORB, BRISK, MSER, GFTT, HARRIS,
     Dense, SimpleBlob, Grid, Pyramid and (SIFT, SURF which are non-free) */
#define FEATURE_DETECTOR                  "PyramidFAST"

/* Available feature extractors: ORB, BRISK, BRIEF
     (SIFT, SURF which are non-free) */
#define FEATURE_DETECTOR_EXTRACTOR        "ORB"


// Optical Flow constants
//#define OPTICAL_FLOW_USE_GREYSCALE_IMAGES
#define OF_MAX_VERROR                   12.0
#define OF_RADIUS_MATCH                  5.0

// Triangulation
#define TRIANGULATION_MAX_ITERATIONS    10
#define TRIANGULATION_EPSILON            0.001

// Point Cloud constants
#define POINT_CLOUD_TRIANGULATION_SEARCH_RADIUS               0.025
#define POINT_CLOUD_TRIANGULATION_MULTIPLIER                  2.5
#define POINT_CLOUD_TRIANGULATION_MAX_NEAREST_NEIGHBOUR     100
#define POINT_CLOUD_TRIANGULATION_MAX_SURFACE_ANGLE        DEG2RAD(45)
#define POINT_CLOUD_TRIANGULATION_MIN_ANGLE                DEG2RAD(10)
#define POINT_CLOUD_TRIANGULATION_MAX_ANGLE                DEG2RAD(120)

#endif

#endif
