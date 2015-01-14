#ifndef SMALL_HELPER_H_
#define SMALL_HELPER_H_

inline bool inRange(float in, float range, float base ){
	if(	(base + range) > in
		&& in > ( base - range)) return true;
	return false;
}

inline float point_distance(pcl::PointXYZI &point){
	return sqrt((point.x* point.x)+(point.y* point.y)+(point.z* point.z));
}

#endif
