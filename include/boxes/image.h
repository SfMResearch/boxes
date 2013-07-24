
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

			// mat
			const cv::Mat* get_mat();

			// descriptors
			cv::Mat* get_descriptors();

			// keypoints
			std::vector<cv::KeyPoint>* get_keypoints();

		private:
			cv::Mat mat;

			// descriptors
			cv::Mat* descriptors;
			cv::Mat* calc_descriptors();

			// keypoints
			std::vector<cv::KeyPoint>* keypoints;
			std::vector<cv::KeyPoint>* calc_keypoints();
	};
};

#endif
