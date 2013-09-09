
#ifndef BOXES_CONSTANTS_H
#define BOXES_CONSTANTS_H

// Match types
#define MATCH_TYPE_NORMAL                  0
#define MATCH_TYPE_RADIUS                  1

// Feature Detector
/* Available feature detectors: FAST, STAR, ORB, BRISK, MSER, GFTT, HARRIS,
     Dense, SimpleBlob, Grid, Pyramid and (SIFT, SURF which are non-free) */

#define FEATURE_DETECTOR_BRISK                    "BRISK"
#define FEATURE_DETECTOR_FAST                     "FAST"
#define FEATURE_DETECTOR_GFTT                     "GFTT"
#define FEATURE_DETECTOR_HARRIS                   "HARRIS"
#define FEATURE_DETECTOR_MSER                     "MSER"
#define FEATURE_DETECTOR_ORB                      "ORB"
#define FEATURE_DETECTOR_PYRAMID_FAST             "PyramidFAST"
#ifdef BOXES_NONFREE
# define FEATURE_DETECTOR_SIFT                    "SIFT"
# define FEATURE_DETECTOR_SURF                    "SURF"

# define DEFAULT_FEATURE_DETECTOR                 "SURF"
#else
# define DEFAULT_FEATURE_DETECTOR                 "PyramidFast"
#endif

/* Available feature extractors: ORB, BRISK, BRIEF
     (SIFT, SURF which are non-free) */
#define FEATURE_DETECTOR_EXTRACTOR_BRIEF          "BRIEF"
#define FEATURE_DETECTOR_EXTRACTOR_BRISK          "BRISK"
#define FEATURE_DETECTOR_EXTRACTOR_ORB            "ORB"
#ifdef BOXES_NONFREE
# define FEATURE_DETECTOR_EXTRACTOR_SIFT          "SIFT"
# define FEATURE_DETECTOR_EXTRACTOR_SURF          "SURF"

# define DEFAULT_FEATURE_DETECTOR_EXTRACTOR       "SURF"
#else
# define DEFAULT_FEATURE_DETECTOR_EXTRACTOR       "ORB"
#endif

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
#define POINT_CLOUD_TRIANGULATION_MULTIPLIER                  3.0
#define POINT_CLOUD_TRIANGULATION_MAX_NEAREST_NEIGHBOUR     200
#define POINT_CLOUD_TRIANGULATION_MAX_SURFACE_ANGLE        DEG2RAD(50)
#define POINT_CLOUD_TRIANGULATION_MIN_ANGLE                DEG2RAD(10)
#define POINT_CLOUD_TRIANGULATION_MAX_ANGLE                DEG2RAD(120)

#endif

#endif
