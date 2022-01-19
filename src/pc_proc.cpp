#include <pc_proc.h>



namespace cvl_toolkit {


	bool operator<(const pt_id& left, const pt_id& right) {
		if (left.x != right.x)return left.x < right.x;
		else return left.y < right.y;
	}

	bool operator>(const pt_id& left, const pt_id& right) {
		if (left.x != right.x)return left.x > right.x;
		else return left.y > right.y;
	}

	int compare(pt_id ptid, cv::Point2f pt) {
		if (ptid.x != pt.x) {
			if (ptid.x < pt.x)return 2;
			else return 0;
		}
		else {
			if (ptid.y != pt.y) {
				if (ptid.y < pt.y)return 2;
				else return 0;
			}
			else return 1;
		}
	}

	void meshGenerater(Eigen::Vector3d center, Eigen::Matrix3d rot, float vertAngle, plyObject& plyData, double threshold) {

		std::vector<cv::Point2f> points;
		std::vector<pt_id> pids;
		int vertexNum = plyData.getVertexNumber();
		//	Mat img = Mat::zeros(SIDELENGTH,SIDELENGTH,CV_8UC3);

		//plot
		float* vp = plyData.getVertecesPointer();

		for (int i = 0; i < vertexNum; i++) {
			double phi;
			double theta;
			Eigen::Vector3d point;
			//transform
			point << vp[i * 3], vp[i * 3 + 1], vp[i * 3 + 2];
			if (point(0) != point(0) || point(1) != point(1) || point(2) != point(2)) {
				phi = 0;
				point(0) = 0;
				continue;
			}
			point = point - center;
			point = rot * point;
			double r = sqrt(point.dot(point));
			if (r < 0.5)continue;
			phi = acos(point(2, 0) / r);
			theta = atan2(point(1, 0), point(0, 0));
			cv::Point2f p;
			if (phi * 180 / M_PI > vertAngle)continue;
			double r2d = (phi * 180 / M_PI / vertAngle)*SIDELENGTH / 2;
			p.x = SIDELENGTH / 2 - r2d * cos(theta);
			p.y = SIDELENGTH / 2 + r2d * sin(theta);
			if (p.x < 0 || p.x >= SIDELENGTH || p.y < 0 || p.y >= SIDELENGTH)continue;
			if (p.x != p.x || p.y != p.y) {
				p.x = 0;
				continue;
			}
			points.push_back(p);
			pt_id pi;
			pi.x = p.x;
			pi.y = p.y;
			pi.idx = i;

			pids.push_back(pi);
			//		int red,green,blue;
			//		red=(i>>16)&0xff;
			//		green=(i>>8)&0xff;
			//		blue=(i>>0)&0xff;
			//		circle(img,p,0,Scalar(red,green,blue));
		}
		sort(pids.begin(), pids.end());
		//using delauney method
		CvMemStorage* storage = cvCreateMemStorage(0);
		cv::Subdiv2D	subdiv;
		subdiv.initDelaunay(cv::Rect(0, 0, SIDELENGTH, SIDELENGTH));

		subdiv.insert(points);
		std::vector<cv::Vec6f> triangles;
		subdiv.getTriangleList(triangles);

		std::vector<Eigen::Vector3i> faceIndeces;
		for (auto it = triangles.begin(); it != triangles.end(); it++) {
			cv::Vec6f &vec = *it;
			cv::Point2f p1(vec[0], vec[1]);
			cv::Point2f p2(vec[2], vec[3]);
			cv::Point2f p3(vec[4], vec[5]);
			if (p1.x >= 0 && p1.x <= SIDELENGTH && p1.y >= 0 && p1.y <= SIDELENGTH && p2.x >= 0 && p2.x <= SIDELENGTH && p2.y >= 0 && p2.y <= SIDELENGTH && p3.x >= 0 && p3.x <= SIDELENGTH && p3.y >= 0 && p3.y <= SIDELENGTH) {
				int index1, index2, index3;
				//			index1=(img.at<Vec3b>(p1.y,p1.x)[0]<<16)+(img.at<Vec3b>(p1.y,p1.x)[1]<<8)+img.at<Vec3b>(p1.y,p1.x)[2];
				//			index2=(img.at<Vec3b>(p2.y,p2.x)[0]<<16)+(img.at<Vec3b>(p2.y,p2.x)[1]<<8)+img.at<Vec3b>(p2.y,p2.x)[2];
				//			index3=(img.at<Vec3b>(p3.y,p3.x)[0]<<16)+(img.at<Vec3b>(p3.y,p3.x)[1]<<8)+img.at<Vec3b>(p3.y,p3.x)[2];
				index1 = search(pids, p1, 0, pids.size() - 1);
				index2 = search(pids, p2, 0, pids.size() - 1);
				index3 = search(pids, p3, 0, pids.size() - 1);
				//			line(img,p1,p2,Scalar(64,255,128));
				//			line(img,p2,p3,Scalar(64,255,128));
				//			line(img,p3,p1,Scalar(64,255,128));
				if (index1 != 0 && index2 != 0 && index3 != 0) {
					//faceのエッジカット
					Eigen::Vector3d p1_, p2_, p3_;
					p1_ << vp[index1 * 3], vp[index1 * 3 + 1], vp[index1 * 3 + 2];
					p2_ << vp[index2 * 3], vp[index2 * 3 + 1], vp[index2 * 3 + 2];
					p3_ << vp[index3 * 3], vp[index3 * 3 + 1], vp[index3 * 3 + 2];
					p1_ = p1_ - center;
					p2_ = p2_ - center;
					p3_ = p3_ - center;
					double aveLength = sqrt(p1_.dot(p1_)) + sqrt(p2_.dot(p2_)) + sqrt(p3_.dot(p3_));
					aveLength /= 3;
					double lp1 = sqrt((p1_ - p2_).dot(p1_ - p2_));
					double lp2 = sqrt((p2_ - p3_).dot(p2_ - p3_));
					double lp3 = sqrt((p3_ - p1_).dot(p3_ - p1_));
					double thres = aveLength * threshold;
					if (lp1 < thres&&lp2 < thres&&lp3 < thres)faceIndeces.push_back(Eigen::Vector3i(index1, index2, index3));
				}

			};
		}

		unsigned int* faces = (unsigned int*)malloc(sizeof(unsigned int) * faceIndeces.size() * 3);
		int cnt = 0;
		for (auto it = faceIndeces.begin(); it != faceIndeces.end(); it++) {
			faces[cnt * 3] = (*it)(0);
			faces[cnt * 3 + 1] = (*it)(1);
			faces[cnt * 3 + 2] = (*it)(2);
			cnt++;
		}
		plyData.setFacePointer(faces, faceIndeces.size());

	}





	int search(std::vector<pt_id>& vec, cv::Point2f& pt, int ids, int ide) {
		if (ids >= ide)return vec.at(ids).idx;
		int id = (ids + ide) / 2;
		switch (compare(vec.at(id), pt)) {
		case 0:
			return search(vec, pt, ids, id - 1);
			break;
		case 1:
			return vec.at(id).idx;
			break;
		case 2:
			return search(vec, pt, id + 1, ide);
			break;
		}
	};





}