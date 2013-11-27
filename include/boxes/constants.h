
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

/*
 * Private definitions that may only be used internally.
 */
#ifdef BOXES_PRIVATE

#include <boxes/converters.h>

//#define FEATURE_MATCHER_REPROJECTION_ERROR
#define REPROJECTION_ERROR_MAX         200.0

#define MATCH_VALID_RATIO                0.8

// Optical Flow constants
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
#define SURF_MIN_HESSIAN               300

#endif

#endif
