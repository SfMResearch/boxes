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

#ifndef BOXES_FEATURE_MATCHER_OPTICAL_FLOW_H
#define BOXES_FEATURE_MATCHER_OPTICAL_FLOW_H

#include <string>

#include <boxes/boxes.h>
#include <boxes/feature_matcher.h>
#include <boxes/image.h>

namespace Boxes {
	class FeatureMatcherOpticalFlow: public FeatureMatcher {
		public:
			FeatureMatcherOpticalFlow(Boxes* boxes, Image *image1, Image *image2):
				FeatureMatcher(boxes, image1, image2) {};
			~FeatureMatcherOpticalFlow() {};

			void match();
			void draw_matches(const std::string filename);
	};
};

#endif
