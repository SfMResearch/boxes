
#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/constants.h>
#include <boxes/converters.h>
#include <boxes/feature_matcher_optical_flow.h>

namespace Boxes {
	void FeatureMatcherOpticalFlow::match() {
		// Remove any stale matches that might be in here.
		this->matches.clear();

		std::vector<cv::Point2f> points1 = convertKeyPoints(this->keypoints1);
		std::vector<cv::Point2f> points2 = convertKeyPoints(this->keypoints2);
		std::vector<cv::Point2f> points2x(points1.size());

#ifdef OPTICAL_FLOW_USE_GREYSCALE_IMAGES
		// Convert images to greyscale.
		cv::Mat greyscale1 = this->image1->get_greyscale_mat();
		cv::Mat greyscale2 = this->image2->get_greyscale_mat();
#else
		cv::Mat greyscale1 = *this->image1->get_mat();
		cv::Mat greyscale2 = *this->image2->get_mat();
#endif

		std::vector<uchar> vstatus;
		std::vector<float> verror;

		// Calculate the optical flow field, i.e. how each point1 moved across the two images
		cv::calcOpticalFlowPyrLK(greyscale1, greyscale2, points1, points2x, vstatus, verror);

		std::vector<MatchPoint> match_points;

		// First, filter out the points with high error.
		for (unsigned int i = 0; i < vstatus.size(); i++) {
			// Save good points with a low error rate.
			if (vstatus[i] && verror[i] < OF_MAX_VERROR) {
				MatchPoint match_point;

				match_point.pt = points2x[i];
				match_point.query_index = i;

				match_points.push_back(match_point);
			}
		}

		std::vector<cv::Point2f> good_points1;
		for (std::vector<MatchPoint>::const_iterator i = match_points.begin(); i != match_points.end(); ++i) {
			good_points1.push_back(i->pt);
		}

		// Format appropriate data structures.
		cv::Mat good_points1_flat = cv::Mat(good_points1).reshape(1, good_points1.size());
		cv::Mat points2_flat = cv::Mat(points2).reshape(1, points2.size());

		this->_match(&good_points1_flat, &points2_flat, &match_points, MATCH_TYPE_RADIUS, CV_L2);
	}
}
