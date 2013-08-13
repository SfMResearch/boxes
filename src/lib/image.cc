
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/image.h>

namespace Boxes {
	Image::Image() {
	}

	Image::Image(const std::string filename) {
		this->mat = cv::imread(filename);

		assert(!this->mat.empty());
	}

	Image::Image(cv::Mat mat) {
		this->mat = mat;
	}

	Image::~Image() {
		// Free keypoint cache.
		if (this->keypoints)
			delete this->keypoints;

		// Free descriptors cache.
		if (this->descriptors)
			delete this->descriptors;
	}

	void Image::show() {
		cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
		cv::imshow("Image", this->mat);
	}

	void Image::write(const std::string filename) {
		cv::imwrite(filename, this->mat);
	}

	const cv::Mat* Image::get_mat() const {
		return &this->mat;
	}

	cv::Size Image::size() const {
		return this->mat.size();
	}

	std::vector<cv::KeyPoint>* Image::calc_keypoints() {
		std::vector<cv::KeyPoint>* output = new std::vector<cv::KeyPoint>();

		cv::FastFeatureDetector detector = cv::FastFeatureDetector();
		detector.detect(this->mat, *output);

		return output;
	}

	const std::vector<cv::KeyPoint>* Image::get_keypoints() {
		if (!this->keypoints)
			this->keypoints = this->calc_keypoints();

		return this->keypoints;
	}

	cv::Mat* Image::calc_descriptors() {
		cv::Mat* descriptors = new cv::Mat();

		// Get keypoints.
		if (!this->keypoints)
			this->keypoints = this->calc_keypoints();

		// Extract descriptors.
		cv::OrbDescriptorExtractor extractor = cv::OrbDescriptorExtractor();
		extractor.compute(this->mat, *this->keypoints, *descriptors);

		return descriptors;
	}

	cv::Mat* Image::get_descriptors() {
		if (!this->descriptors)
			this->descriptors = this->calc_descriptors();

		return this->descriptors;
	}

	cv::Mat Image::get_greyscale_mat() {
		if (this->mat.channels() == 1) {
			return this->mat;
		}

		cv::Mat greyscale;
		cv::cvtColor(this->mat, greyscale, CV_RGB2GRAY);

		return greyscale;
	}

	cv::Mat Image::guess_camera_matrix() const {
		cv::Size image_size = this->size();

		cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
		camera_matrix.at<double>(0, 0) = image_size.width;
		camera_matrix.at<double>(1, 1) = image_size.height;
		camera_matrix.at<double>(2, 2) = 1.0;

		// The center of the image.
		camera_matrix.at<double>(0, 2) = image_size.width / 2;
		camera_matrix.at<double>(1, 2) = image_size.height / 2;

		return camera_matrix;
	}
};
