#define _USE_MATH_DEFINES
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "Eigen/Eigen"
#include "Eigen/Core"
#include <time.h> 

#if defined(WIN32) || defined(WIN64)
// Windows 32-bit and 64-bit
#include <Windows.h>
#include <windows.system.h>
#include <imagehlp.h>
#pragma comment(lib, "imagehlp.lib")

#elif defined(MAC_OSX)
// Mac OSX

#else
// Linux and all others
// Using GCC 4 where hiding attributes is possible
#include <sys/stat.h>

#endif

#ifndef CVL_UTIL
#define CVL_UTIL

namespace cvl_toolkit {

	//Euler object
	struct _6dof {
		double rx, ry, rz;
		double x, y, z;
		friend std::ostream& operator<<(std::ostream& os, const _6dof& dt);
	};

	//file operation


	namespace conversion {
		//double
		Eigen::Vector4d dcm2q(Eigen::Matrix3d& dcm);
		Eigen::Matrix3d q2dcm(Eigen::Vector4d& q);
		Eigen::Matrix3d axisRot2R(double rx, double ry, double rz);
		Eigen::Matrix3d ladybug_rot2xyz(double rph[3]);
		void R2axisRot(Eigen::Matrix3d R, double& rx, double& ry, double& rz);

		Eigen::Matrix4d _6dof2m(_6dof dof);
		_6dof m2_6dof(Eigen::Matrix4d& m);
		void _6dof2trans_quaternion(_6dof dof, Eigen::Vector3d& trans, Eigen::Vector4d& quaternion);
		void mat2axis_angle(Eigen::Matrix3d m, Eigen::Vector3d& retv, double& angle);
		Eigen::Matrix3d axis_angle2mat(Eigen::Vector3d axis, double anglerad);

		Eigen::Matrix3f faxisRot2R(double rx, double ry, double rz);

		void panorama_pix2bearing(double u,double v,int imageWidth,int imageHeight,Eigen::Vector3d& ret);
		void panorama_bearing2pix(Eigen::Vector3d bv,int imageWidth,int imageHeight,double&u,double& v);
		void xyz2polar(Eigen::Vector3d bv, double&phi, double& theta);
	};

	bool headString(std::string line, std:: string chara);
	Eigen::Vector3d getNorm(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3);
	Eigen::Matrix4d getMatrixFlomPly(std::string fn);
	void HSVAngle2Color(double radangle, unsigned char* rgb);
};

#endif