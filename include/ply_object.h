#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility.h>

#include "Eigen/Core"
#include "Eigen/Eigen"
#include "Eigen/Dense"


#ifndef PLYOBJ
#define PLYOBJ

typedef std::map<int, int> IDXMAP;
namespace cvl_toolkit {
	class plyObject {
	private:
		typedef struct {
			unsigned char	nindex_;
			unsigned int			index0_, index1_, index2_;
		}ply_index;


	protected:
		Eigen::Matrix4d GlobalPose;
		Eigen::Vector3d g;
		float* verteces;
		float* norm;
		float* reflectance;
		unsigned int* faces;
		unsigned char* rgba;//{r,g,b,a,r,g,b,a,...}
#if defined(_WIN32) || defined(_WIN64)
		__int64 facenum;
		__int64 vertexnum;
#elif defined(MAC_OSX)
		long facenum;
		long vertexnum;
#elif defined(__unix__)
		long facenum;
		long vertexnum;
#endif

		double alpha;
		bool isANN;
		bool bRead;

		bool bG = false;
		bool bC = false;
	public:

		enum PROP {
			PROP_XF,
			PROP_YF,
			PROP_ZF,
			PROP_XD,
			PROP_YD,
			PROP_ZD,
			PROP_INTENSITY,
			PROP_R,
			PROP_G,
			PROP_B,
			PROP_A,
			PROP_OTHER_D,
			PROP_OTHER_F,
			PROP_OTHER_UC
		};
		plyObject() {
			vertexnum = 0;
			facenum = 0;
			bRead = false;
			norm = NULL;
			GlobalPose = Eigen::Matrix4d::Identity();
		}

		//io
		bool readPlyFile(std::string fileName);

		void writePlyFile(std::string fileName);
		void writePlyFileRGBForMeshlab(std::string fileName);

		//info
		bool isReflectance() { return bG; }
		bool isColor() { return bC; }
		int getVertexNumber() { return vertexnum; };
		int getFaceNumber() { return facenum; };

		float* getVertecesPointer() { return verteces; };
		float* getReflectancePointer() { return reflectance; };
		unsigned char* getRgbaPointer() { return rgba; };
		unsigned int* getFaces() { return faces; };

		float* getNormPointer() { return norm; };


		void setVertecesPointer(float* vertices, int vtnum) { verteces = vertices; vertexnum = vtnum; };
		void setRgbaPointer(unsigned char* rgba_, int vtnum) { rgba = rgba_; vertexnum = vtnum; bC = true; };
		void setReflectancePointer(float* reflectance_, int vtnum) { reflectance = reflectance_; vertexnum = vtnum; bG = true; };
		void setFacePointer(unsigned int* faces_, int fcnum) { faces = faces_; facenum = fcnum; };


		//3d utils	
		Eigen::Vector3d getCentroid() { return g; };
		void computeNorm();
		void removeNoMeshPoints();
		void removeZeroPoints();
		void transform(Eigen::Matrix4d m);


		void release();
	};
}

#endif