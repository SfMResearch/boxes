
#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/constants.h>
#include <boxes/converters.h>
#include <boxes/feature_matcher_optical_flow.h>

namespace Boxes {
	void FeatureMatcherOpticalFlow::match() {
		// Remove any stale matches that might be in here.
		this->matches.clear();

#ifdef OPTICAL_FLOW_ALGO_FARNEBACK
		cv::Mat flow;
		cv::Mat greyscale1 = *this->image1->get_greyscale_mat();
		cv::Mat greyscale2 = *this->image2->get_greyscale_mat();

		cv::calcOpticalFlowFarneback(greyscale1, greyscale2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

		for (int y = 0; y < flow.rows; y++) {
			for (int x = 0; x < flow.cols; x++) {
				cv::Point2f f = flow.at<cv::Point2f>(y, x);

				MatchPoint mp;
				mp.pt1 = cv::Point2f(x, y);
				mp.pt2 = cv::Point2f(x + f.x, y + f.y);
				this->matches.push_back(mp);
			}
		}

		this->calculate_fundamental_matrix();

#else
		// Convert images to greyscale.
		cv::Mat greyscale1 = *this->image1->get_greyscale_mat();
		cv::Mat greyscale2 = *this->image2->get_greyscale_mat();

		std::vector<MatchPoint> match_points;

# ifdef OPTICAL_FLOW_USE_GFTT
		std::vector<cv::Point2f> points1 = this->image1->get_good_features_to_track();
		std::vector<cv::Point2f> points2 = this->image2->get_good_features_to_track();
# else
		std::vector<cv::KeyPoint>* keypoints1 = this->image1->get_keypoints();
		std::vector<cv::KeyPoint>* keypoints2 = this->image2->get_keypoints();

		std::vector<cv::Point2f> points1 = convertKeyPoints(keypoints1);
		std::vector<cv::Point2f> points2 = convertKeyPoints(keypoints2);
# endif
		std::vector<cv::Point2f> points2x(points1.size());

		cv::Size search_window_size = cv::Size(OF_SEARCH_WINDOW_SIZE, OF_SEARCH_WINDOW_SIZE);

		std::vector<uchar> vstatus;
		std::vector<float> verror;

		// Calculate the optical flow field, i.e. how each point1 moved across the two images
		cv::calcOpticalFlowPyrLK(greyscale1, greyscale2, points1, points2x, vstatus, verror,
			search_window_size, OF_MAX_PYRAMIDS);

		// First, filter out the points with high error.
		for (unsigned int i = 0; i < vstatus.size(); i++) {
			// Save good points with a low error rate.
			if (vstatus[i] && verror[i] < OF_MAX_VERROR) {
				MatchPoint match_point;
				match_point.pt1 = points1[i];
				match_point.pt2 = points2x[i];

				match_points.push_back(match_point);
			}
		}

		std::vector<cv::Point2f> good_points1;
		for (std::vector<MatchPoint>::const_iterator i = match_points.begin(); i != match_points.end(); ++i) {
			good_points1.push_back(i->pt1);
		}

		// Format appropriate data structures.
		cv::Mat good_points1_flat = cv::Mat(good_points1).reshape(1, good_points1.size());
		cv::Mat points2_flat = cv::Mat(points2).reshape(1, points2.size());

		this->_match(&good_points1_flat, &points2_flat, &match_points, MATCH_TYPE_RADIUS, CV_L2);
#endif
	}

	void FeatureMatcherOpticalFlow::draw_matches(const std::string filename) {
		const cv::Scalar colour1 = CV_RGB(0, 255, 0);
		const cv::Scalar colour2 = CV_RGB(0, 0, 255);

		cv::Mat img_matches = cv::Mat(*this->image1->get_mat());

		const int step = 4;

		int counter = 0;
		for (MatchPoint mp: this->matches) {
			if (counter++ % step)
				continue;

			cv::line(img_matches, mp.pt1, mp.pt2, colour1);
			cv::circle(img_matches, mp.pt1, 2, colour2, -1);
		}

		Image image = Image(this->boxes, img_matches);
		image.write(filename);
	}
}
