
#ifndef BOXES_FEATURE_MATCHER_OPTICAL_FLOW_H
#define BOXES_FEATURE_MATCHER_OPTICAL_FLOW_H

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
	};
};

#endif
