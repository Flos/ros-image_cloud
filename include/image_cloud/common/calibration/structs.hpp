/*
 * structs.h
 *
 *  Created on: 28.01.2015
 *      Author: fnolden
 */

#include <assert.h>
#include <tf/tf.h>

#ifndef INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_
#define INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_

namespace search
{
const std::string spacer = "\t";

struct Value_calculator{
	float min;
	float max;
	int steps_max;
	float step_width;
	float center;

	Value_calculator(){
		init_range(0, 0, 1);
	};

	Value_calculator( float value, float range, int steps_max = 3){
		init_min_max(value - range/2, value + range/2, steps_max);
		this->center = value;
	}

	void init_range( float value, float range, int steps_max = 3){
		init_min_max(value - range/2, value + range/2, steps_max);
		this->center = value;
	}

	void init_min_max(float min, float max, int steps_max = 3){
		assert(min <= max);
		assert(steps_max > 0);

		this->min = min;
		this->max = max;
		this->steps_max = steps_max;
		if(steps_max != 1){
			step_width = (max - min)/(steps_max-1); // First step = min, last step = max
		}
		else{
			step_width = 0;
		}
		this->center = (max - min);
	}

	float at(int step){
		assert(step < steps_max);
		return min + (step_width*step);
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "min:" << spacer << min << spacer;
		ss << "center:" << spacer << center << spacer;
		ss << "max:" << spacer << max << spacer;
		ss << "steps:" << spacer << steps_max << spacer;
		ss << "step_width:"<< spacer << step_width;
		return ss.str();
	}

	std::string to_description_string(){
		std::stringstream ss;
		ss << "min:" << spacer;
		ss << "center:" << spacer;
		ss << "max:" << spacer;
		ss << "steps:" << spacer;
		ss << "step_width:";
		return ss.str();
	}

	std::string to_simple_string(){
		std::stringstream ss;
		ss << min << spacer;
		ss << center << spacer;
		ss << max << spacer;
		ss << steps_max << spacer;
		ss << step_width;
		return ss.str();
	}
};

struct Search_value{
	Search_value(){
		init(0, 0, 0, 0, 0, 0, 0);
	}

	Search_value(tf::Transform tf, long unsigned int result = 0){
		init(tf, result);
	}

	Search_value( float x, float y, float z, float roll, float pitch, float yaw, long unsigned int result = 0){
		init(x, y, z, roll, pitch, yaw, result);
	}

	void init(float x, float y, float z, float roll, float pitch, float yaw, long unsigned int result = 0)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->score = result;
	}

	void init(tf::Transform tf, long unsigned int result = 0)
	{
		this->x = tf.getOrigin()[0];
		this->y = tf.getOrigin()[1];
		this->z = tf.getOrigin()[2];

		double r,p,y;
		tf.getBasis().getRPY(r, p, y);
		this->roll = r;
		this->pitch = p;
		this->yaw = y;
		this->score = result;
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "score:" << spacer << score << spacer;
		ss << "x:" << spacer << x << spacer;
		ss << "y:" << spacer << y << spacer;
		ss <<" z:" << spacer << z << spacer;
		ss << "roll:" << spacer << roll << spacer;
		ss << "pitch:" << spacer << pitch << spacer;
		ss << "yaw:" << spacer << yaw;

		return ss.str();
	}

	std::string to_description_string(){
		std::stringstream ss;
		ss << "score:" << spacer;
		ss << "x:" << spacer;
		ss << "y:" << spacer;
		ss <<" z:" << spacer;
		ss << "roll:" << spacer;
		ss << "pitch:" << spacer;
		ss << "yaw:";

		return ss.str();
	}

	std::string to_simple_string(){
		std::stringstream ss;
		ss << score << spacer;
		ss << x << spacer;
		ss << y << spacer;
		ss << z << spacer;
		ss << roll << spacer;
		ss << pitch << spacer;
		ss << yaw;
		return ss.str();
	}

	tf::Transform get_transform(){
		tf::Transform tf;

		get_transform(tf);

		return tf;
	}

	void get_transform(tf::Transform &tf){
		tf.setOrigin( tf::Vector3( x, y, z ) );

		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw );
		tf.setRotation( q );
	}

	Search_value& operator=(const Search_value& a)
	{
		x=a.x;
		y=a.y;
		z=a.x;
		roll=a.roll;
		pitch=a.pitch;
		yaw=a.yaw;
		score=a.score;
		return *this;
	}

	Search_value operator+(const Search_value& a) const
	{
		return Search_value(a.x+x, a.y+y, a.z+z, a.roll+roll, a.pitch+pitch, a.yaw+yaw, a.score+score);
	}

	Search_value operator-(const Search_value& a) const
	{
		return Search_value( x - a.x, y - a.y, z - a.z, roll - a.roll, pitch - a.pitch, yaw - a.yaw, score - a.score);
	}

	// equality comparison. doesn't modify object. therefore const.
	bool operator==(const Search_value& a) const
	{
		return (x == a.x && y == a.y && z == a.z && roll == a.roll && pitch == a.pitch && yaw == a.yaw && score == a.score);
	}


	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	long unsigned int score;
};

struct Search_setup{
	Value_calculator x;
	Value_calculator y;
	Value_calculator z;
	Value_calculator roll;
	Value_calculator pitch;
	Value_calculator yaw;

	Search_setup(){
		init(0,0,0,0,0,0,0,1);
	}

	Search_setup(tf::Transform center, float *ranges, int *steps){
		this->x.init_range(center.getOrigin()[0], ranges[0], steps[0]);
		this->y.init_range(center.getOrigin()[1], ranges[1], steps[1]);
		this->z.init_range(center.getOrigin()[2], ranges[2], steps[2]);
		double rpy[3];
		center.getBasis().getRPY(rpy[0], rpy[1], rpy[2]);

		this->roll.init_range(rpy[0], ranges[3], steps[3]);
		this->pitch.init_range(rpy[1], ranges[4], steps[4]);
		this->yaw.init_range(rpy[2], ranges[5], steps[5]);
	}

	Search_setup(tf::Transform center, std::vector<float> ranges, std::vector<int> steps){
		this->x.init_range(center.getOrigin()[0], ranges[0], steps[0]);
		this->y.init_range(center.getOrigin()[1], ranges[1], steps[1]);
		this->z.init_range(center.getOrigin()[2], ranges[2], steps[2]);
		double rpy[3];
		center.getBasis().getRPY(rpy[0], rpy[1], rpy[2]);

		this->roll.init_range(rpy[0], ranges[3], steps[3]);
		this->pitch.init_range(rpy[1], ranges[4], steps[4]);
		this->yaw.init_range(rpy[2], ranges[5], steps[5]);
	}

	Search_setup(Search_value val, float* ranges, float* steps){
		this->x.init_range(val.x, ranges[0], steps[0]);
		this->y.init_range(val.y, ranges[1], steps[1]);
		this->z.init_range(val.z, ranges[2], steps[2]);
		this->roll.init_range(val.roll, ranges[3], steps[3]);
		this->pitch.init_range(val.pitch, ranges[4], steps[4]);
		this->yaw.init_range(val.yaw, ranges[5], steps[5]);
	}

	Search_setup(Search_value val, float range, float step){
			this->x.init_range(val.x, range, step);
			this->y.init_range(val.y, range, step);
			this->z.init_range(val.z, range, step);
			this->roll.init_range(val.roll, range, step);
			this->pitch.init_range(val.pitch, range, step);
			this->yaw.init_range(val.yaw, range, step);
	}

	Search_setup(float* pose, float *ranges, float *steps){
		this->x.init_range(pose[0], ranges[0], steps[0]);
		this->y.init_range(pose[1], ranges[1], steps[1]);
		this->z.init_range(pose[2], ranges[2], steps[2]);
		this->roll.init_range(pose[3], ranges[3], steps[3]);
		this->pitch.init_range(pose[4], ranges[4], steps[4]);
		this->yaw.init_range(pose[5], ranges[5], steps[5]);
	}

	Search_setup(float x, float y, float z, float roll, float pitch, float yaw, float range, int steps){
			init( x, y, z, roll, pitch, yaw, range, steps);
		}

	void init(float x, float y, float z, float roll, float pitch, float yaw, float range, int steps){
		this->x.init_range(x, range, steps);
		this->y.init_range(y, range, steps);
		this->z.init_range(z, range, steps);
		this->roll.init_range(roll, range, steps);
		this->pitch.init_range(pitch, range, steps);
		this->yaw.init_range(yaw, range, steps);
	}

	std::string to_description_string(){
			std::stringstream ss;
			ss << x.to_description_string() << spacer;
			ss << y.to_description_string() << spacer;
			ss << z.to_description_string() << spacer;
			ss << roll.to_description_string() << spacer;
			ss << pitch.to_description_string() << spacer;
			ss << yaw.to_description_string();
			return ss.str();
	}

	std::string to_simple_string(){
		std::stringstream ss;
		ss << x.to_simple_string() << spacer;
		ss << y.to_simple_string() << spacer;
		ss << z.to_simple_string() << spacer;
		ss << roll.to_simple_string() << spacer;
		ss << pitch.to_simple_string() << spacer;
		ss << yaw.to_simple_string();
		return ss.str();
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "x:" << spacer << x.to_string() << spacer;
		ss << "y:" << spacer << y.to_string() << spacer;
		ss << "z:" << spacer << z.to_string() << spacer;
		ss << "roll:" << spacer << roll.to_string() << spacer;
		ss << "pitch:" << spacer << pitch.to_string() << spacer;
		ss << "yaw:" << spacer << yaw.to_string();
		return ss.str();
	}
};


inline bool sort_by_score(const Search_value &lhs, const Search_value &rhs) { return lhs.score > rhs.score; }

struct Multi_search_result{
	std::vector<Search_value> best_results;
	Search_value center;
	Search_value best;

	unsigned int size(){
		return best_results.size();
	}

	Search_value at(unsigned int idx){
		return best_results.at(idx);
	}

	long unsigned int nr_worse;
	long unsigned int nr_same;

	Multi_search_result(){
		init();
	}

	Multi_search_result(std::vector<Search_value> &results, Search_value &center){
		init();
		evaluate(results);
		this->center = center;
	}

	void evaluate(const std::vector<Search_value> &results){
		//Sort results by score
		best_results.assign(results.begin(), results.end());
		std::sort(best_results.begin(), best_results.end(), sort_by_score);
		best = best_results.at(0);

		assert(best.score >= best_results.at(best_results.size()-1).score);
		assert(best.score >= best_results.at(0).score);
		assert(best.score >= center.score);

		nr_same = 0;
		nr_worse = 0;

		for(int i = 1; i < best_results.size(); ++i){
			if( best.score == best_results.at(i).score )
			{
				++nr_same;
			}
			else if( best.score > best_results.at(i).score){
				nr_worse = size()-i;
				break;
			}
		}

		assert(nr_worse + nr_same + 1 == size());
	}

	void init(Search_value in, Search_value best, long unsigned int nr_worse, long unsigned int nr_same = 0){
		this->size();
		this->nr_worse = nr_worse;
		this->center = in;
		this->best = best;
		this->nr_same = nr_same;
	}

	void init(){
		nr_worse = 0;
		nr_same = 0;
		Search_value empty;
		empty.init(0,0,0,0,0,0,0);
		center = empty;
		best = empty;
	}

	float get_fc(){
		if(size() > 0){
			return (float)nr_worse/(float)size();
		}
		return 0;
	}


	Search_value get_delta_best(){
		return best-center;
	}

	std::string to_description_string(){
		std::stringstream ss;
		ss << "total:" << spacer;
		ss << "worse:" << spacer;
		ss << "same:" << spacer;
		ss << "fc:" << spacer;
		ss << center.to_description_string() << spacer;
		ss << best.to_description_string() << spacer;
		ss << get_delta_best().to_description_string();

		return ss.str();
	}

	std::string to_simple_string(){
		std::stringstream ss;
		ss << size() << spacer;
		ss << nr_worse << spacer;
		ss << nr_same << spacer;
		ss << get_fc() << spacer;
		ss << center.to_simple_string() << spacer;
		ss << best.to_simple_string()<< spacer;
		ss << get_delta_best().to_simple_string();

		return ss.str();
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "total:" << spacer << size() << spacer;
		ss << "worse:" << spacer << nr_worse << spacer;
		ss << "same:" << spacer << nr_same << spacer;
		ss << "fc:" << spacer << get_fc() << spacer;
		ss << "in:" <<spacer << center.to_string() << spacer;
		ss << "out:" << spacer << best.to_string() << spacer;
		ss << "delta:" << spacer << get_delta_best().to_string();

		return ss.str();
	}
};

struct Window{
	int window_size;
	std::deque<cv::Mat> images;
	std::deque<pcl::PointCloud<pcl::PointXYZI> > pointclouds;

	void check(){
		assert(images.size() == pointclouds.size());

		if(size() > window_size){
			pop_front();
		}
	}

	size_t size(){
		check();
		return images.size();
	}

	void pop_front(){
		images.pop_front();
		pointclouds.pop_front();

		check();
	}

	void push_back(cv::Mat image, pcl::PointCloud<pcl::PointXYZI> pointcloud){
		images.push_back(image);
		pointclouds.push_back(pointcloud);

		check();
	}
};

}
#endif /* INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_ */
