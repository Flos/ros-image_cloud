#ifndef SMALL_HELPER_H_
#define SMALL_HELPER_H_

template <class T>
inline bool inRange(T in, T range, T base ){
	if(	(base - range) < in && in < ( base + range) ) return true;
	return false;
}

inline float point_distance(pcl::PointXYZI &point){
	return sqrt((point.x* point.x)+(point.y* point.y)+(point.z* point.z));
}

template <class T>
inline bool between(T min, T test, T max){
	if( min < test && test < max ) return true;
	return false;
}

#endif
