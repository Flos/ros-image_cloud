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
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(Filter3d,
		(OFF)
		(DEPTH)
		(DEPTH_INTENSITY)
		(DEPTH_EDGE)
		(NORMAL_DIFF)
		(RANGE_BORDERS)
		(DEPTH_NEIGHBORS)
		(DEPTH_EDGE_PROJECTION)
		(HIT_SAME_POINT)
		(DEPTH_RADIUS)
		(OTHER)
		(REMOVE_CLUSTER_2D)
		(DEPTH_INTENSITY_AND_REMOVE_CLUSER_2D)
		(DEPTH_INTENSITY_NORMAL_DIFF)
		(REMOVE_CLUSER_2D_RADIUS_SEARCH)
		(EDGE_IMAGE_PLANE)
		(EDGE_IMAGE_PLANE_2D_RADIUS_SEARCH)
		(EDGE_IMAGE_PLANE_NORMAL_DIFF)
		(DEPTH_EDGE_PROJECTION_AGGREGATED)
		(DEPTH_NEIGHBOR_DISONTINUITY)
		(NR_ENUMS))
};

namespace image_filter{

	namespace blur{
		DEFINE_ENUM_WITH_STRING_CONVERSIONS(Blur,
			(OFF)
			(BILATERAL)
			(BLUR)
			(GAUSSIAN)
			(MEDIAN)
			(NR_ENUMS))
	};

	namespace edge{
		DEFINE_ENUM_WITH_STRING_CONVERSIONS(Edge,
			(OFF)
			(CANNY)
			(LAPLACE)
			(MAX)
			(NR_ENUMS))
	};

	namespace enlight{
		DEFINE_ENUM_WITH_STRING_CONVERSIONS(Enlight,
			(OFF)
			(ON)
			(NR_ENUMS))
	};
};

#endif /* INCLUDE_IMAGE_CLOUD_COMMON_FILTER_PCL_ENUMS_H_ */
