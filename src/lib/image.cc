
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/image.h>

namespace Boxes {
	BoxesImage::BoxesImage() {
	}

	BoxesImage::BoxesImage(const std::string filename) {
		this->mat = cv::imread(filename);

		assert(!this->mat.empty());
	}

	BoxesImage::BoxesImage(cv::Mat mat) {
		this->mat = mat;
	}

	BoxesImage::~BoxesImage() {
		// Free keypoint cache.
		if (this->keypoints)
			delete this->keypoints;

		// Free descriptors cache.
		if (this->descriptors)
			delete this->descriptors;
	}

	void BoxesImage::show() {
		cv::namedWindow("BoxesImage", CV_WINDOW_AUTOSIZE);
		cv::imshow("BoxesImage", this->mat);
	}

	void BoxesImage::write(const std::string filename) {
		cv::imwrite(filename, this->mat);
	}

	const cv::Mat* BoxesImage::get_mat() {
		return &this->mat;
	}

	std::vector<cv::KeyPoint>* BoxesImage::calc_keypoints() {
		std::vector<cv::KeyPoint>* output = new std::vector<cv::KeyPoint>();

		cv::OrbFeatureDetector detector = cv::OrbFeatureDetector();
		detector.detect(this->mat, *output);

		return output;
	}

	std::vector<cv::KeyPoint>* BoxesImage::get_keypoints() {
		if (!this->keypoints)
			this->keypoints = this->calc_keypoints();

		return this->keypoints;
	}

	cv::Mat* BoxesImage::calc_descriptors() {
		cv::Mat* descriptors = new cv::Mat();

		// Get keypoints.
		std::vector<cv::KeyPoint>* keypoints = this->get_keypoints();

		// Extract descriptors.
		cv::OrbDescriptorExtractor extractor = cv::OrbDescriptorExtractor();
		extractor.compute(this->mat, *keypoints, *descriptors);

		return descriptors;
	}

	cv::Mat* BoxesImage::get_descriptors() {
		if (!this->descriptors)
			this->descriptors = this->calc_descriptors();

		return this->descriptors;
	}
};
