#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/filter/cv/edge.hpp>
#include <image_cloud/common/project2d.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>


#ifndef FILTER3D_DEPTH_EDGE_H_
#define FILTER3D_DEPTH_EDGE_H_

namespace filter_3d
{


template <typename PointT>
inline
void get_diff(int col, int row, int col_c, int row_c, float &result,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &in) {

	if (between(-1, col, (int)in.size()) && between(-1, row, (int)in[0].size())) {
		if(in[col][row]){
			//printf("i: %d %d,c: %d %d, - ", col, row, col_c, row_c);
			result = in[col_c][row_c]->z - in[col][row]->z;
		}
	}
}


template <typename PointT>
inline
void get_diff_neighbors_rows(int row, int col, int steps, int col_c, int row_c, float &result,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &in){

	for(int r = 0; r < steps; ++r)
	{
		get_diff<PointT>(col, row + r, col_c, row_c, result, in);
	}
}

template <typename PointT>
inline
void get_diff_neighbors_cols(int row, int col, int steps, int col_c, int row_c, float &result,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &in){

	for(int c = 0; c < steps; ++c)
	{
		get_diff<PointT>(col + c, row, col_c, row_c, result, in);
	}
}

template <typename PointT>
inline
void find_neighbor(int start_row, int start_col, int step_row, int step_col, int steps_max, float &result,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &in){

	for(int c = 1; c < steps_max; ++c)
	{
		int col = start_col+c*step_col;
		int row = start_row+c*step_row;
		if (between(-1, col, (int)in.size()) && between(-1, row, (int)in[0].size())) {
			if(in[col][row]){
				result = in[col][row]->z;
				break;
			}
		}
	}
}

template <typename PointT>
inline
float max_diff_neighbors(int row_c, int col_c, int neighbors,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &in){
	//Check
	float result = 0;

//	get_diff<PointT>(col_c -1, row_c -1, col_c, row_c, result, in);
//	get_diff<PointT>(col_c	  , row_c -1, col_c, row_c, result, in);
//	get_diff<PointT>(col_c +1, row_c -1, col_c, row_c, result, in);

	int steps = 4;
	float prev,current,next;
	find_neighbor(row_c, col_c, 0, -1, steps, prev, in );

	current = in[col_c][row_c]->z;
	find_neighbor(row_c, col_c, 0, +1, steps, next, in );

	if(prev < current -.45 && inRange<float>(next, 0.15, current)){
		result = current - prev;
	}
	if(next < current -.45 && inRange<float>(prev, 0.15, current)){
		result = std::max(current - next, result);
	}

//	get_diff<PointT>(col_c -1, row_c   , col_c, row_c, result, in);
//	get_diff<PointT>(col_c +1, row_c   , col_c, row_c, result, in);

//	get_diff<PointT>(col_c -1, row_c +1, col_c, row_c, result, in);
//	get_diff<PointT>(col_c   , row_c +1, col_c, row_c, result, in);
//	get_diff<PointT>(col_c +1, row_c +1, col_c, row_c, result, in);

//	if(neighbors > 1){
//		// increase kernel
//		for(int i = 2; i < neighbors; ++i){
//			// Top col
////			get_diff_neighbors_cols<PointT>(col_c - i, row_c - i, neighbors*2 +1, col_c, row_c, result, in);
////			// Bottem col
////			get_diff_neighbors_cols<PointT>(col_c - i, row_c + i, neighbors*2 +1, col_c, row_c, result, in);
//
//			// Left row
//			//get_diff_neighbors_rows<PointT>(col_c - i, row_c - i, neighbors*2 -1, col_c, row_c, result, in);
//			// Right row
//			//get_diff_neighbors_rows<PointT>(col_c + i, row_c - i, neighbors*2 -1, col_c, row_c, result, in);
//		}
//
//	}

	return result;
}


template <typename PointT>
inline void
depth_edge(std::vector<std::vector<boost::shared_ptr<PointT> > > &in,
		pcl::PointCloud<PointT> &out,
		int neighbors = 1,
		float range = 0.3)
{
	for(int r = 0; r < in[0].size(); r++)
	{
		std::deque<boost::shared_ptr<PointT> > points;
		for(int c = 0; c < in.size(); c++)
		{
			if(in[c][r]){
				points.push_back(in[c][r]);
				if(points.size() == 2){
					// Check for any interesting changes
					if( (points[0]->intensity > in[c][r]->intensity )// 9,(4),7,(4),(4),9, 2,1,4,6,7
							)
					{
						out.push_back( *in[c][r] );
						points.pop_front();
					}
				}
//				if( max_diff_neighbors<PointT>(r, c, neighbors, in) > range ){
//					out.push_back( *in[c][r] );
//					printf("added: %f %f %f %f\n", in[c][r]->x, in[c][r]->y, in[c][r]->z, in[c][r]->intensity);
//				}
			}
		}
	}

	//printf("rows: %lu cols: %lu neighbors %d, range: %f\n", in[0].size(), in.size(), neighbors, range);
}


}

#endif
