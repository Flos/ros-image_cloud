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

template <typename type>
struct Value_calculator{
	type min;
	type max;
	int steps_max;
	type step_width;
	type center;

	Value_calculator(){
		init_range(0, 0, 1);
	};

	Value_calculator( type value, type range, int steps_max = 3){
		init_min_max(value - range/2, value + range/2, steps_max);
		this->center = value;
	}

	void init_range( type value, type range, int steps_max = 3){
		init_min_max(value - range/2, value + range/2, steps_max);
		this->center = value;
	}

	void init_min_max(type min, type max, int steps_max = 3){
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

	type at(int step){
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

template <typename type>
struct Search_value_6d{
	Search_value_6d(){
		init(0, 0, 0, 0, 0, 0, 0, 0);
	}

	Search_value_6d(tf::Transform tf, long unsigned int result = 0, long unsigned int points = 0){
		init(tf, result, points);
	}

	Search_value_6d( type x, type y, type z, type roll, type pitch, type yaw, long unsigned int score = 0, long unsigned int points = 0){
		init(x, y, z, roll, pitch, yaw, score, points);
	}

	void init(type x, type y, type z, type roll, type pitch, type yaw, long unsigned int score = 0, long unsigned int points = 0)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->score = score;
		this->points = points;
	}

	void init(tf::Transform tf, long unsigned int result = 0, long unsigned int points = 0)
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
		this->points = points;
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "score:" << spacer << score << spacer;
//		ss << "points: " << spacer << points << spacer;
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
//		ss << "points:" << spacer;
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
//		ss << points << spacer;
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

	double get_score_per_point()
	{
		if(points == 0 || score ==0) return 0;
		return ((double)score)/(double(points));
	}

	void get_transform(tf::Transform &tf){
		tf.setOrigin( tf::Vector3( x, y, z ) );

		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw );
		tf.setRotation( q );
	}

	Search_value_6d& operator=(const Search_value_6d& a)
	{
		x=a.x;
		y=a.y;
		z=a.z;
		roll=a.roll;
		pitch=a.pitch;
		yaw=a.yaw;
		score=a.score;
		points=a.points;
		return *this;
	}

	Search_value_6d operator+(const Search_value_6d& a) const
	{
		return Search_value_6d(a.x+x, a.y+y, a.z+z, a.roll+roll, a.pitch+pitch, a.yaw+yaw, a.score+score, a.points+points);
	}

	Search_value_6d operator-(const Search_value_6d& a) const
	{
		return Search_value_6d( x - a.x, y - a.y, z - a.z, roll - a.roll, pitch - a.pitch, yaw - a.yaw, score - a.score, a.points - points);
	}

	bool operator < (const Search_value_6d& a) const
	{
		return (score < a.score);
	}

	bool operator > (const Search_value_6d& a) const
	{
		return (score > a.score);
	}

	// equality comparison. doesn't modify object. therefore const.
	bool operator==(const Search_value_6d& a) const
	{
		return (x == a.x && y == a.y && z == a.z && roll == a.roll && pitch == a.pitch && yaw == a.yaw && score == a.score && points == a.points);
	}

	bool operator!=(const Search_value_6d& a) const
	{
		return (x != a.x || y != a.y || z != a.z || roll != a.roll || pitch != a.pitch || yaw != a.yaw || score != a.score || points != a.points);
	}


	type x;
	type y;
	type z;
	type roll;
	type pitch;
	type yaw;
	long unsigned int score;
	long unsigned int points;
};

typedef Search_value_6d<double> search_value_d;
typedef Search_value_6d<float> search_value_f;
typedef search_value_d search_value;

typedef std::vector<search_value_d> search_value_vector_d;
typedef std::vector<search_value_f> search_value_vector_f;
typedef search_value_vector_d search_value_vector;


template <typename type>
struct Search_setup{
	Value_calculator<type> x;
	Value_calculator<type> y;
	Value_calculator<type> z;
	Value_calculator<type> roll;
	Value_calculator<type> pitch;
	Value_calculator<type> yaw;

	Search_setup(){
		init(0,0,0,0,0,0,0,1);
	}

	Search_setup(tf::Transform center, type *ranges, int *steps){
		this->x.init_range(center.getOrigin()[0], ranges[0], steps[0]);
		this->y.init_range(center.getOrigin()[1], ranges[1], steps[1]);
		this->z.init_range(center.getOrigin()[2], ranges[2], steps[2]);
		double rpy[3];
		center.getBasis().getRPY(rpy[0], rpy[1], rpy[2]);

		this->roll.init_range(rpy[0], ranges[3], steps[3]);
		this->pitch.init_range(rpy[1], ranges[4], steps[4]);
		this->yaw.init_range(rpy[2], ranges[5], steps[5]);
	}

	Search_setup(tf::Transform center, std::vector<type> ranges, std::vector<int> steps){
		this->x.init_range(center.getOrigin()[0], ranges[0], steps[0]);
		this->y.init_range(center.getOrigin()[1], ranges[1], steps[1]);
		this->z.init_range(center.getOrigin()[2], ranges[2], steps[2]);
		double rpy[3];
		center.getBasis().getRPY(rpy[0], rpy[1], rpy[2]);

		this->roll.init_range(rpy[0], ranges[3], steps[3]);
		this->pitch.init_range(rpy[1], ranges[4], steps[4]);
		this->yaw.init_range(rpy[2], ranges[5], steps[5]);
	}

	Search_setup(Search_value_6d<type> val, std::vector<type> ranges, std::vector<int> steps){
		this->x.init_range(val.x, ranges[0], steps[0]);
		this->y.init_range(val.y, ranges[1], steps[1]);
		this->z.init_range(val.z, ranges[2], steps[2]);

		this->roll.init_range(val.roll, ranges[3], steps[3]);
		this->pitch.init_range(val.pitch, ranges[4], steps[4]);
		this->yaw.init_range(val.yaw, ranges[5], steps[5]);
	}

	Search_setup(Search_value_6d<type> val, type* ranges, type* steps){
		this->x.init_range(val.x, ranges[0], steps[0]);
		this->y.init_range(val.y, ranges[1], steps[1]);
		this->z.init_range(val.z, ranges[2], steps[2]);
		this->roll.init_range(val.roll, ranges[3], steps[3]);
		this->pitch.init_range(val.pitch, ranges[4], steps[4]);
		this->yaw.init_range(val.yaw, ranges[5], steps[5]);
	}

	Search_setup(Search_value_6d<type> val, type range, type step){
			this->x.init_range(val.x, range, step);
			this->y.init_range(val.y, range, step);
			this->z.init_range(val.z, range, step);
			this->roll.init_range(val.roll, range, step);
			this->pitch.init_range(val.pitch, range, step);
			this->yaw.init_range(val.yaw, range, step);
	}

	Search_setup(type* pose, type *ranges, type *steps){
		this->x.init_range(pose[0], ranges[0], steps[0]);
		this->y.init_range(pose[1], ranges[1], steps[1]);
		this->z.init_range(pose[2], ranges[2], steps[2]);
		this->roll.init_range(pose[3], ranges[3], steps[3]);
		this->pitch.init_range(pose[4], ranges[4], steps[4]);
		this->yaw.init_range(pose[5], ranges[5], steps[5]);
	}

	Search_setup(type x, type y, type z, type roll, type pitch, type yaw, type range, int steps){
			init( x, y, z, roll, pitch, yaw, range, steps);
		}

	void init(type x, type y, type z, type roll, type pitch, type yaw, type range, int steps){
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

typedef Search_setup<double> search_setup_d;
typedef Search_setup<float> search_setup_f;
typedef search_setup_d search_setup;

typedef std::vector<search_setup_d> search_setup_vector_d;
typedef std::vector<search_setup_f> search_setup_vector_f;
typedef search_setup_vector_d search_setup_vector;


template <typename search_value_type>
struct Multi_search_result{

	std::vector<search_value_type> best_results;
	search_value_type center;
	search_value_type best;

	unsigned int size(){
		return best_results.size();
	}

	search_value_type at(unsigned int idx){
		return best_results.at(idx);
	}

	long unsigned int nr_worse;
	long unsigned int nr_same;

	Multi_search_result(){
		init();
	}

	Multi_search_result(std::vector<search_value_type> &results, search_value_type &center){
		init();
		evaluate(results);
		this->center = center;
	}

	void evaluate(const std::vector<search_value_type> &results){

		best_results = results;

		std::sort(best_results.begin(), best_results.end(), std::greater<search_value_type>());

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

	void init(search_value_type in, search_value_type best, long unsigned int nr_worse, long unsigned int nr_same = 0){
		this->size();
		this->nr_worse = nr_worse;
		this->center = in;
		this->best = best;
		this->nr_same = nr_same;
	}

	void init(){
		nr_worse = 0;
		nr_same = 0;
		search_value_type empty;
		empty.init(0,0,0,0,0,0,0);
		center = empty;
		best = empty;
	}

	double get_fc(){
		if(size() > 0){
			return (double)nr_worse/(double)size();
		}
		return 0;
	}


	search_value_type get_delta_best(){
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


typedef Multi_search_result<search_value_d> multi_search_results_d;
typedef Multi_search_result<search_value_f> multi_search_results_f;
typedef multi_search_results_d multi_search_results;



template <typename multi_search_value_type, typename search_value_type>
struct Multi_search_results_vector{
	std::vector<multi_search_value_type> data;


	multi_search_value_type& at(long unsigned int idx){
		return data.at(idx);
	}

	long unsigned int size(){
		return data.size();
	}

	void push_back(multi_search_value_type value){
		data.push_back(value);
	}

	search_value_type get_best(){

		search_value_type value;
		get_best_search_value(value);

		return value;
	}

	void get_best_search_value(search_value_type &best_result){
		for(int i=0; i < size(); ++i){
			if(best_result.score < at(i).best.score){
				best_result = at(i).best;
			}
		}
	}
};

typedef Multi_search_results_vector<multi_search_results_d, search_value_d> multi_search_results_vector_d;
typedef Multi_search_results_vector<multi_search_results_f, search_value_f> multi_search_results_vector_f;
typedef multi_search_results_vector_d multi_search_results_vector;




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
