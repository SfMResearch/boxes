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

#ifndef BOXES_CLOUD_POINT_H
#define BOXES_CLOUD_POINT_H

#include <opencv2/opencv.hpp>

#include <boxes/image.h>

namespace Boxes {
	class CloudPoint {
		public:
			CloudPoint();

			cv::Point3d pt;
			double reprojection_error;

			cv::Point2f pt1;
			cv::Point2f pt2;

			bool operator==(const CloudPoint& other) const;

			// colours
			void set_colour_from_image(Image* image);
			float get_colour(float def = 0xffffff) const;

		private:
			float colour = -1.0;
	};
};

#endif
