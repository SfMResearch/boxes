
#ifndef BOXES_IMAGE_H
#define BOXES_IMAGE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Boxes {
	class BoxesImage {
		public:
			BoxesImage();
			BoxesImage(const std::string filename);
			BoxesImage(cv::Mat mat);
			~BoxesImage();

			// Basic methods.
			void show();
			void write(const std::string filename);
			cv::Size size() const;

			// mat
			const cv::Mat* get_mat() const;
			cv::Mat get_greyscale_mat();

			// descriptors
			cv::Mat* get_descriptors();

			// keypoints
			const std::vector<cv::KeyPoint>* get_keypoints();

			// camera matrix
			cv::Mat guess_camera_matrix() const;

		private:
			cv::Mat mat;

			// descriptors
			cv::Mat* descriptors = NULL;
			cv::Mat* calc_descriptors();

			// keypoints
			std::vector<cv::KeyPoint>* keypoints = NULL;
			std::vector<cv::KeyPoint>* calc_keypoints();
	};
};

#endif
