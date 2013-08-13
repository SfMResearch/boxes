
#ifndef BOXES_FEATURE_MATCHER_OPTICAL_FLOW_H
#define BOXES_FEATURE_MATCHER_OPTICAL_FLOW_H

#include <boxes/feature_matcher.h>
#include <boxes/image.h>

namespace Boxes {
	class FeatureMatcherOpticalFlow: public FeatureMatcher {
		public:
			FeatureMatcherOpticalFlow(BoxesImage *image1, BoxesImage *image2):
				FeatureMatcher(image1, image2) {};

		private:
			void match();
	};
};

#endif
