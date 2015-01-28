#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <common/type.hpp>
#include <math.h>
#include <common/calibration/multi_score.hpp>
#include <common/calibration/structs.hpp>

#ifndef SEARCH_GRID_6_DH_
#define SEARCH_GRID_6D_H_

namespace search
{

struct Value{
	float min;
	float max;
	int step_count;
	float step_width;

	Value( float value, float range, int step_count = 3){
		init_min_max(value - range/2, value + range/2, step_count);
	}

	void init_range( float value, float range, int step_count = 3){
		init_min_max(value - range/2, value + range/2, step_count);
	}

	void init_min_max(float min, float max, int step_count = 3){
		assert(min < max);
		assert(step_count > 1);
		this->min = min;
		this->max = max;
		this->step_count = step_count;
		step_width = (max - min)/step_count;
	}

	float at(int step){
		assert(step < step_count);
		return min + (step_width*step);
	}
};

struct Search_setup{
	Value x;
	Value y;
	Value z;
	Value roll;
	Value pitch;
	Value yaw;
};

struct Search_value{
	Search_value(){
		init(0, 0, 0, 0, 0, 0, 0);
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
		this->result = result;
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "x: " << x << " y: " << y <<" z: " << z;
		ss << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw;
		ss << "result: " << result << "\n";
		return ss.str();
	}

	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	long unsigned int result;
};


	void grid_setup(Search_setup setup, std::vector<Search_value>& results){
		for(int x=0; x < setup.x.step_count; ++x)
		{
			for(int y=0; y < setup.y.step_count; ++x)
			{
				for(int z=0; z < setup.z.step_count; ++x)
				{
					for(int roll=0; roll < setup.roll.step_count; ++x)
					{
						for(int pitch=0; pitch < setup.pitch.step_count; ++x)
						{
							for(int yaw=0; yaw < setup.yaw.step_count; ++x)
							{
								results.push_back(Search_value(setup.x.at(x), setup.x.at(y), setup.x.at(z), setup.x.at(roll), setup.x.at(pitch), setup.x.at(yaw)));
							}
						}
					}
				}
			}
		}
	}

	template <typename PointT, typename ImageT>
	void grid_calculate(const image_geometry::PinholeCameraModel &camera_model, const std::vector<pcl::PointCloud<PointT> > &pointclouds, const std::vector<cv::Mat> &edge_images, std::vector<Search_value>& results){
		for(int i=0; i < results; ++i){
			multi_score<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));
		}
	}
}
