/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

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
