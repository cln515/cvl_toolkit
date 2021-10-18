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
	};























}