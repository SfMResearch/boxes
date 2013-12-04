
#include <opencv2/opencv.hpp>

#include <boxes/cloud_point.h>

namespace Boxes {
	CloudPoint::CloudPoint() {
	}

	bool CloudPoint::operator==(const CloudPoint& other) const {
		return this->pt == other.pt;
	}

	float CloudPoint::get_colour(float def) const {
		if (this->colour < 0) {
			return def;
		}

		return this->colour;
	}

	void CloudPoint::set_colour_from_image(Image* image) {
		const cv::Mat* mat = image->get_mat();

		cv::Vec3b pixel = mat->at<cv::Vec3b>(this->pt2);
		uint32_t rgb = ((uint32_t)pixel[2] << 16 | (uint32_t)pixel[1] << 8 | (uint32_t)pixel[0]);

		this->colour = *reinterpret_cast<float*>(&rgb);
	}
};
