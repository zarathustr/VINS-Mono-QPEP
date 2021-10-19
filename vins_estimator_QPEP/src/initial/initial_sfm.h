#pragma once 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <LibQPEP/utils.h>
#include <LibQPEP/pnp_WQD.h>
#include <LibQPEP/solver_WQ_1_2_3_4_5_9_13_17_33_49_approx.h>
#include <LibQPEP/QPEP_grobner.h>
#include <LibQPEP/QPEP_lm_single.h>
#include <LibQPEP/misc_pnp_funcs.h>


inline std::vector<cv::Point3d> Vector3dToPoint3d(std::vector<Eigen::Vector3d> pt)
{
    std::vector<cv::Point3d> tmp;
    for(int i = 0; i < pt.size(); ++i)
    {
        Eigen::Vector3d point = pt[i];
        cv::Point3d vec;
        vec.x = point(0);
        vec.y = point(1);
        vec.z = point(2);
        tmp.push_back(vec);
    }
    return tmp;
}

inline std::vector<cv::Point2d> Vector2dToPoint2d(std::vector<Eigen::Vector2d> pt)
{
    std::vector<cv::Point2d> tmp;
    for(int i = 0; i < pt.size(); ++i)
    {
        Eigen::Vector2d point = pt[i];
        cv::Point2d vec;
        vec.x = point(0);
        vec.y = point(1);
        tmp.push_back(vec);
    }
    return tmp;
}


inline std::vector<Eigen::Vector3d> Point3fToVector3d(std::vector<cv::Point3f> pt)
{
    std::vector<Eigen::Vector3d> tmp;
    for(int i = 0; i < pt.size(); ++i)
    {
        cv::Point3f point = pt[i];
        Eigen::Vector3d vec;
        vec(0) = point.x;
        vec(1) = point.y;
        vec(2) = point.z;
        tmp.push_back(vec);
    }
    return tmp;
}

inline std::vector<Eigen::Vector2d> Point2fToVector2d(std::vector<cv::Point2f> pt)
{
    std::vector<Eigen::Vector2d> tmp;
    for(int i = 0; i < pt.size(); ++i)
    {
        cv::Point2f point = pt[i];
        Eigen::Vector2d vec;
        vec(0) = point.x;
        vec(1) = point.y;
        tmp.push_back(vec);
    }
    return tmp;
}

template <typename T>
std::vector<size_t> sort_indices(const std::vector<T> &v) {
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    std::sort(idx.begin(), idx.end(),
              [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
}


using namespace Eigen;
using namespace std;



struct SFMFeature
{
    bool state;
    int id;
    vector<pair<int,Vector2d>> observation;
    double position[3];
    double depth;
};

struct ReprojectionError3D
{
	ReprojectionError3D(double observed_u, double observed_v)
		:observed_u(observed_u), observed_v(observed_v)
		{}

	template <typename T>
	bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
	{
		T p[3];
		ceres::QuaternionRotatePoint(camera_R, point, p);
		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
		T xp = p[0] / p[2];
    	T yp = p[1] / p[2];
    	residuals[0] = xp - T(observed_u);
    	residuals[1] = yp - T(observed_v);
    	return true;
	}

	static ceres::CostFunction* Create(const double observed_x,
	                                   const double observed_y) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionError3D, 2, 4, 3, 3>(
	          	new ReprojectionError3D(observed_x,observed_y)));
	}

	double observed_u;
	double observed_v;
};

class GlobalSFM
{
public:
	GlobalSFM();
	bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);
    void solvePnP_QPEP(const std::vector<cv::Point3f> &pts_3_vector,
                       const vector<cv::Point2f> &pts_2_vector,
                       const Eigen::Matrix3d &K__,
                       cv::Mat &R, cv::Mat &t, bool useInitial = false);

private:
	bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);

	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
							Vector2d &point0, Vector2d &point1, Vector3d &point_3d);
	void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
							  int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
							  vector<SFMFeature> &sfm_f);


	int feature_num;
};