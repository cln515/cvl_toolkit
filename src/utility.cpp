#include <utility.h>

namespace cvl_toolkit {


	std::ostream& operator<<(std::ostream& os, const _6dof& dof)
	{
		os << dof.rx << ',' << dof.ry << ',' << dof.rz << ',' << dof.x << ',' << dof.y << ',' << dof.z;
		return os;
	}

	namespace conversion {
		//double
		Eigen::Vector4d dcm2q(Eigen::Matrix3d& dcm) {
			Eigen::Vector4d q;
			if (dcm.trace() > 0) {
				double sr = sqrt(1 + dcm.trace());
				double sr2 = sr * 2;

				q(0) = (dcm(1, 2) - dcm(2, 1)) / sr2;
				q(1) = (dcm(2, 0) - dcm(0, 2)) / sr2;
				q(2) = (dcm(0, 1) - dcm(1, 0)) / sr2;
				q(3) = 0.5*sr;
			}
			else {
				if (dcm(0, 0) > dcm(1, 1) && dcm(0, 0) > dcm(2, 2)) {
					double sr = sqrt(1 + (dcm(0, 0) - (dcm(1, 1) + dcm(2, 2))));
					double sr2 = sr * 2;
					q(3) = (dcm(1, 2) - dcm(2, 1)) / sr2;
					q(2) = (dcm(2, 0) + dcm(0, 2)) / sr2;
					q(1) = (dcm(0, 1) + dcm(1, 0)) / sr2;
					q(0) = 0.5*sr;
				}
				else if (dcm(1, 1) > dcm(2, 2)) {
					double  sr = sqrt(1 + (dcm(1, 1) - (dcm(2, 2) + dcm(0, 0))));
					double  sr2 = 2 * sr;
					q(0) = (dcm(1, 0) + dcm(0, 1)) / sr2;
					q(1) = 0.5 * sr;
					q(2) = (dcm(1, 2) + dcm(2, 1)) / sr2;
					q(3) = (dcm(2, 0) - dcm(0, 2)) / sr2;

				}
				else {
					double  sr = sqrt(1 + (dcm(2, 2) - (dcm(0, 0) + dcm(1, 1))));
					double  sr2 = 2 * sr;
					q(0) = (dcm(2, 0) + dcm(0, 2)) / sr2;
					q(1) = (dcm(1, 2) + dcm(2, 1)) / sr2;
					q(2) = 0.5 * sr;
					q(3) = (dcm(0, 1) - dcm(1, 0)) / sr2;
				}
			}
			return q;
		};
		Eigen::Matrix3d q2dcm(Eigen::Vector4d& q) {
			Eigen::Matrix3d R;

			// Build quaternion element products
			double q1q1 = q(0)*q(0);
			double q1q2 = q(0)*q(1);
			double q1q3 = q(0)*q(2);
			double q1q4 = q(0)*q(3);

			double q2q2 = q(1)*q(1);
			double q2q3 = q(1)*q(2);
			double q2q4 = q(1)*q(3);

			double q3q3 = q(2)*q(2);
			double q3q4 = q(2)*q(3);

			double q4q4 = q(3)*q(3);

			// Build DCM
			R(0, 0) = q1q1 - q2q2 - q3q3 + q4q4;
			R(0, 1) = 2 * (q1q2 + q3q4);
			R(0, 2) = 2 * (q1q3 - q2q4);

			R(1, 0) = 2 * (q1q2 - q3q4);
			R(1, 1) = -q1q1 + q2q2 - q3q3 + q4q4;
			R(1, 2) = 2 * (q2q3 + q1q4);

			R(2, 0) = 2 * (q1q3 + q2q4);
			R(2, 1) = 2 * (q2q3 - q1q4);
			R(2, 2) = -q1q1 - q2q2 + q3q3 + q4q4;

			return R;

		};
		Eigen::Matrix3d axisRot2R(double rx, double ry, double rz) {
			Eigen::Matrix4d R, rotx, roty, rotz;
			double sinv, cosv;
			sinv = sin(rx), cosv = cos(rx);



			rotx << 1, 0, 0, 0, 0, cosv, -sinv, 0, 0, sinv, cosv, 0, 0, 0, 0, 1;
			sinv = sin(ry); cosv = cos(ry);
			roty << cosv, 0, sinv, 0, 0, 1, 0, 0, -sinv, 0, cosv, 0, 0, 0, 0, 1;
			sinv = sin(rz); cosv = cos(rz);
			rotz << cosv, -sinv, 0, 0, sinv, cosv, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
			R = rotx * roty*rotz;
			Eigen::Matrix3d retMat = R.block(0, 0, 3, 3);
			return retMat;
		};
		Eigen::Matrix3d ladybug_rot2xyz(double rph[3]) {
			double sr, sp, sh, cr, cp, ch;
			sr = sin(rph[0]); cr = cos(rph[0]);
			sp = sin(rph[1]); cp = cos(rph[1]);
			sh = sin(rph[2]); ch = cos(rph[2]);

			Eigen::Matrix3d R;
			R << ch * cp, -sh * cr + ch * sp*sr, sh*sr + ch * sp*cr,
				sh*cp, ch*cr + sh * sp*sr, -ch * sr + sh * sp*cr,
				-sp, cp*sr, cp*cr;
			return R;
		};
		void R2axisRot(Eigen::Matrix3d R, double& rx, double& ry, double& rz) {
			ry = asin(R(0, 2));
			rx = -atan2(R(1, 2), R(2, 2));
			rz = -atan2(R(0, 1), R(0, 0));
		};

		Eigen::Matrix4d _6dof2m(_6dof dof) {
			Eigen::Matrix4d ret;
			ret.block(0, 0, 3, 3) = axisRot2R(dof.rx, dof.ry, dof.rz);
			ret(0, 3) = dof.x;
			ret(1, 3) = dof.y;
			ret(2, 3) = dof.z;
			ret.block(3, 0, 1, 4) << 0, 0, 0, 1;
			return ret;
		};;
		_6dof m2_6dof(Eigen::Matrix4d& m) {
			Eigen::Matrix3d r = m.block(0, 0, 3, 3);
			_6dof dof;
			R2axisRot(r, dof.rx, dof.ry, dof.rz);
			dof.x = m(0, 3);
			dof.y = m(1, 3);
			dof.z = m(2, 3);
			return dof;
		};
		void _6dof2trans_quaternion(_6dof dof, Eigen::Vector3d& trans, Eigen::Vector4d& quaternion) {
			trans << dof.x, dof.y, dof.z;
			double s1 = sin(dof.rx), s2 = sin(dof.ry), s3 = sin(dof.rz), c1 = cos(dof.rx), c2 = cos(dof.ry), c3 = cos(dof.rz);
			quaternion << s1 * s2*c3 + c1 * c2*s3,
				s1*c2*c3 + c1 * s2*s3,
				c1*s2*c3 - s1 * c2*s3,
				c1*c2*c3 - s1 * s2*s3;
		};
		void mat2axis_angle(Eigen::Matrix3d m, Eigen::Vector3d& retv, double& angle) {
			double x, y, z;
			double r = sqrt((m(2, 1) - m(1, 2))*(m(2, 1) - m(1, 2)) + (m(0, 2) - m(2, 0))*(m(0, 2) - m(2, 0)) + (m(1, 0) - m(0, 1))*(m(1, 0) - m(0, 1)));
			x = (m(2, 1) - m(1, 2)) / r;
			y = (m(0, 2) - m(2, 0)) / r;
			z = (m(1, 0) - m(0, 1)) / r;
			Eigen::Vector3d t;
			t << x, y, z;
			retv = t;
			angle = acos((m(0, 0) + m(1, 1) + m(2, 2) - 1) / 2);
		};
		Eigen::Matrix3d axis_angle2mat(Eigen::Vector3d axis, double angle){
			Eigen::Matrix3d skewMat;
			skewMat << 0, -axis(2), axis(1),
				axis(2), 0, -axis(0),
				-axis(1), axis(0), 0;
			return Eigen::Matrix3d::Identity() + sin(angle)*skewMat + (1 - cos(angle))*skewMat*skewMat;
		};





		Eigen::Matrix3f faxisRot2R(double rx, double ry, double rz) {
			Eigen::Matrix4f R, rotx, roty, rotz;
			float sinv, cosv;
			sinv = sin(rx), cosv = cos(rx);



			rotx << 1, 0, 0, 0, 0, cosv, -sinv, 0, 0, sinv, cosv, 0, 0, 0, 0, 1;
			sinv = sin(ry); cosv = cos(ry);
			roty << cosv, 0, sinv, 0, 0, 1, 0, 0, -sinv, 0, cosv, 0, 0, 0, 0, 1;
			sinv = sin(rz); cosv = cos(rz);
			rotz << cosv, -sinv, 0, 0, sinv, cosv, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
			R = rotx * roty*rotz;
			Eigen::Matrix3f retMat = R.block(0, 0, 3, 3);
			return retMat;
		};

		void panorama_pix2bearing(double u, double v,int imageWidth,int imageHeight, Eigen::Vector3d& ret) {
			double theta = -(u*(M_PI * 2) / imageWidth - M_PI);
			double phi = v / imageHeight * M_PI;
			ret << sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi);
		}
		void panorama_bearing2pix(Eigen::Vector3d bv, int imageWidth, int imageHeight, double&u, double& v) {
			double r = bv.norm();
			double theta = atan2(bv(1), bv(0));
			double phi = acos(bv(2) / r);
			u = (-theta + M_PI)* imageWidth / (M_PI * 2);
			v = phi / M_PI * imageHeight;
		}
		
		void xyz2polar(Eigen::Vector3d bv, double&phi, double& theta) {
			double r = bv.norm();
			theta = atan2(bv(1), bv(0));
			phi = acos(bv(2) / r);
		}

	};

	bool headString(std::string line, std::string chara) {
		if (line.find(chara) == 0)return true;
		else return false;
	}

	Eigen::Vector3d getNorm(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
		double v1x = p2(0) - p1(0), v1y = p2(1) - p1(1), v1z = p2(2) - p1(2),
			v2x = p3(0) - p2(0), v2y = p3(1) - p2(1), v2z = p3(2) - p2(2);
		Eigen::Vector3d nc_;
		Eigen::Vector3d nc;
		nc_(0) = v1y * v2z - v1z * v2y;
		nc_(1) = v1z * v2x - v1x * v2z;
		nc_(2) = v1x * v2y - v1y * v2x;
		nc_.normalize();
		nc(0) = nc_(0);
		nc(1) = nc_(1);
		nc(2) = nc_(2);
		return nc;
	};


	Eigen::Matrix4d getMatrixFlomPly(std::string fn) {
		std::ifstream ifs(fn, std::ios::binary);
		std::string line;
		Eigen::Matrix4d globalPose = Eigen::Matrix4d::Identity();
		int n = 0;
		while (getline(ifs, line)) {
			//		ofs<<line<<endl;
			std::cout << line << std::endl;
			if (headString(line, "matrix")) {
				float f[4];
				int i;
				for (i = 0; i < 5; i++) {
					if (i != 0)f[i - 1] = stof(line.substr(0, line.find_first_of(" ")));
					line.erase(0, line.find_first_of(" ") + 1);
				}
				globalPose(0, n) = f[0];
				globalPose(1, n) = f[1];
				globalPose(2, n) = f[2];
				globalPose(3, n) = f[3];
				n++;
			}
			if (headString(line, "end_header"))break;

		}
		ifs.close();
		return globalPose;
	}

	void HSVAngle2Color(double radangle, unsigned char* rgb) {
		double pi_sixtydig = M_PI / 3;
		double angle = ((radangle / (M_PI * 2)) - floor((radangle / (M_PI * 2))))*(M_PI * 2);
		if (angle >= 0 && angle < pi_sixtydig) {
			double val = (angle - pi_sixtydig * 0) / pi_sixtydig;
			rgb[0] = 255;
			rgb[1] = 255 * val;
			rgb[2] = 0;
		}
		else if (angle >= pi_sixtydig * 1 && angle < pi_sixtydig * 2) {
			double val = (angle - pi_sixtydig * 1) / pi_sixtydig;
			rgb[0] = 255 * (1 - val);
			rgb[1] = 255;
			rgb[2] = 0;
		}
		else if (angle >= pi_sixtydig * 2 && angle < pi_sixtydig * 3) {
			double val = (angle - pi_sixtydig * 2) / pi_sixtydig;
			rgb[0] = 0;
			rgb[1] = 255;
			rgb[2] = 255 * (val);
		}
		else if (angle >= pi_sixtydig * 3 && angle < pi_sixtydig * 4) {
			double val = (angle - pi_sixtydig * 3) / pi_sixtydig;
			rgb[0] = 0;
			rgb[1] = 255 * (1 - val);
			rgb[2] = 255;
		}
		else if (angle >= pi_sixtydig * 4 && angle < pi_sixtydig * 5) {
			double val = (angle - pi_sixtydig * 4) / pi_sixtydig;
			rgb[0] = 255 * (val);
			rgb[1] = 0;
			rgb[2] = 255;
		}
		else if (angle >= pi_sixtydig * 5 && angle < pi_sixtydig * 6) {
			double val = (angle - pi_sixtydig * 5) / pi_sixtydig;
			rgb[0] = 255;
			rgb[1] = 0;
			rgb[2] = 255 * (1 - val);
		}


	}
}