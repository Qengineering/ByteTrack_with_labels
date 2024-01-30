#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;
typedef Eigen::Matrix<float, 1, 128, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, 128, Eigen::RowMajor> FEATURESS;
//typedef std::vector<FEATURE> FEATURESS;

//Kalmanfilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 8, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

//main
using RESULT_DATA = std::pair<int, DETECTBOX>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t {
	std::vector<MATCH_DATA> matches;
	std::vector<int> unmatched_tracks;
	std::vector<int> unmatched_detections;
}TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;

//----------------------------------------------------------------------------------
// With a fixed number of elements, there is no need for a std::vector.
// std::vectors require more overhead and are slower than a simple stuct.
//----------------------------------------------------------------------------------
struct Ttlbr
{
    float t;    ///< top
    float l;    ///< left
    float b;    ///< botton
    float r;    ///< right
    int   c;    ///< object class (e.g. COCO)
};
//----------------------------------------------------------------------------------
struct Ttlwh
{
    float t;    ///< top
    float l;    ///< left
    float w;    ///< width
    float h;    ///< height
};
//---------------------------------------------------------------------------
inline std::vector<float> tlwh_to_xyah(Ttlwh tlwh_tmp)
{
	std::vector<float> tlwh_output(4);

    tlwh_output[0]=tlwh_tmp.t+tlwh_tmp.w/2;
    tlwh_output[1]=tlwh_tmp.l+tlwh_tmp.h/2;
    tlwh_output[2]=tlwh_tmp.w/tlwh_tmp.h;
    tlwh_output[3]=tlwh_tmp.h;

	return tlwh_output;
}
//----------------------------------------------------------------------------------
