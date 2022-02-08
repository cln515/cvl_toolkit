#include <ply_object.h>
#include <opencv2/opencv.hpp>

#define SIDELENGTH 20000


#ifndef PCPROC
#define PCPROC
namespace cvl_toolkit {
	struct pt_id {
		float x, y;
		int idx;
	};
	int search(std::vector<pt_id>& vec, cv::Point2f& pt, int ids, int ide);
	void meshGenerater(Eigen::Vector3d center, Eigen::Matrix3d rot, float vertAngle, plyObject& plyData, double threshold);
	void goodFeatureToTrack_onProjection(cv::Mat image, std::vector<cv::Point2f> proj_points, std::vector<int>& selectedIdx, double minDistance, int maxCorners, double cornerThres = 0.0);
}

#endif