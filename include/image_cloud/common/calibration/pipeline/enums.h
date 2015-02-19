/*
 * enums.h
 *
 *  Created on: 19.02.2015
 *      Author: fnolden
 */

#ifndef INCLUDE_IMAGE_CLOUD_COMMON_FILTER_PCL_ENUMS_H_
#define INCLUDE_IMAGE_CLOUD_COMMON_FILTER_PCL_ENUMS_H_

#include <boost/preprocessor.hpp>

#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \
enum name {                                                               \
	BOOST_PP_SEQ_ENUM(enumerators)                                        \
};                                                                        \
inline const char* ToString(name v)                                       \
{                                                                         \
	switch (v)                                                            \
	{                                                                     \
		BOOST_PP_SEQ_FOR_EACH(                                            \
			X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \
			name,                                                         \
			enumerators                                                   \
		)                                                                 \
		default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \
	}                                                                     \
}

namespace pcl_filter{
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(Filter3d, (OFF)(DEPTH)(DEPTH_INTENSITY)(DEPTH_EDGE)(NORMAL_DIFF)(RANGE_BORDERS)(DEPTH_RADIUS)(DEPTH_NEIGHBORS)(DEPTH_EDGE_PROJECTION)(OTHER))
//	enum Filter3d
//	{
//		OFF = 0,
//		DEPTH = 1,
//		DEPTH_INTENSITY = 2,
//		DEPTH_EDGE = 3,
//		NORMAL_DIFF = 4,
//		RANGE_BORDERS = 5,
//		DEPTH_RADIUS = 6,
//		DEPTH_NEIGHBORS = 7,
//		DEPTH_EDGE_PROJECTION = 8,
//		OTHER = 9
//	};
};

namespace image_filter{

	namespace blur{
		enum Blur{
			OFF = 0,
			BILATERAL = 1,
			BLUR = 2,
			GAUSSIAN = 3,
			MEDIAN = 4
		};
	};

	namespace edge{
		enum Edge{
			OFF = 0,
			CANNY = 1,
			LAPLACE = 2,
			MAX = 3
		};
	};

	namespace enlight{
		enum Enlight{
			OFF = 0
		};
	};
};

#endif /* INCLUDE_IMAGE_CLOUD_COMMON_FILTER_PCL_ENUMS_H_ */
