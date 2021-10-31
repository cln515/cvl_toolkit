#include <ply_object.h>

namespace cvl_toolkit {
	bool plyObject::readPlyFile(std::string fileName) {
		std::ifstream ifs(fileName, std::ios::binary);
		std::string line;
		std::string format;
		int n = 0;
		int xi = -1, yi = -1, zi = -1;
		int ri = -1, redi = -1, bluei = -1, greeni = -1;
		int paranum = 0;
		int vertex = 0;
		int face = 0;
		int matIdx = 0;
		std::vector<PROP> props;
		std::vector<int> paramidx;
		while (getline(ifs, line)) {
			std::cout << line << std::endl;
			if (headString(line, "format")) {
				format = line.erase(0, 7);
			}
			if (headString(line, "end_header"))break;
			if (headString(line, "property")) {

				line.erase(0, 9);
				if (headString(line, "float")) {
					line.erase(0, 6);
					if (headString(line, "x")) {
						xi = paranum; props.push_back(PROP_XF); paramidx.push_back(xi);
					}
					else if (headString(line, "y")) {
						yi = paranum; props.push_back(PROP_YF); paramidx.push_back(yi);
					}
					else if (headString(line, "z")) {
						zi = paranum; props.push_back(PROP_ZF); paramidx.push_back(zi);
					}
					else if (headString(line, "intensity")) {
						ri = paranum; props.push_back(PROP_INTENSITY); paramidx.push_back(ri);
					}
					else if (headString(line, "confidence"));
					//				else return false;
					paranum += 4;
				}
				else if (headString(line, "double")) {
					line.erase(0, 7);
					if (headString(line, "x")) {
						xi = paranum; props.push_back(PROP_XD); paramidx.push_back(xi);
					}
					else if (headString(line, "y")) {
						yi = paranum; props.push_back(PROP_YD); paramidx.push_back(yi);
					}
					else if (headString(line, "z")) {
						zi = paranum; props.push_back(PROP_ZD); paramidx.push_back(zi);
					}
					else if (headString(line, "intensity")) {
						ri = paranum; props.push_back(PROP_INTENSITY); paramidx.push_back(ri);
					}
					else if (headString(line, "confidence"));
					//else return false;
					paranum += 8;
				}
				else if (headString(line, "uchar")) {
					line.erase(0, 6);
					if (headString(line, "red")) {
						redi = paranum; props.push_back(PROP_R); paramidx.push_back(redi);
					}
					else if (headString(line, "blue")) {
						bluei = paranum; props.push_back(PROP_B); paramidx.push_back(bluei);
					}
					else if (headString(line, "green")) {
						greeni = paranum; props.push_back(PROP_G); paramidx.push_back(greeni);
					}
					//else if (headString(line, "intensity"))ri = paranum;
					//else if (headString(line, "confidence"));
					paranum += 1;
				}
				//			else return false;

			}
			if (headString(line, "element")) {
				line.erase(0, 8);
				if (headString(line, "vertex")) {
					line.erase(0, 7);
					vertex = stoi(line);

				}
				if (headString(line, "face")) {
					line.erase(0, 5);
					face = stoi(line);
				}
			}
			if (headString(line, "matrix")) {
				float f[4];

				for (int i = 0; i < 5; i++) {
					line.erase(0, line.find_first_not_of(" "));
					if (i > 0)f[i - 1] = stof(line.substr(0, line.find_first_of(" ")));
					line.erase(0, line.find_first_of(" "));
				}
				GlobalPose(0, matIdx) = f[0];
				GlobalPose(1, matIdx) = f[1];
				GlobalPose(2, matIdx) = f[2];
				GlobalPose(3, matIdx) = f[3];
				matIdx++;
			}
		}

		if (xi >= yi || yi >= zi)return false;
		float a = 0;
		int time = 0;
		int num = 0;
		verteces = (float*)malloc(sizeof(float)*vertex * 3);
		faces = (unsigned int*)malloc(sizeof(unsigned int)*(face * 3));

		if (ri >= 0) {
			reflectance = (float*)malloc(sizeof(float)*vertex);
			bG = true;
		}
		if (redi >= 0) {
			rgba = (unsigned char*)malloc(sizeof(char)*(vertex * 4));
			bC = true;
		}


		if (headString(format, "binary_little_endian")) {
			std::ifstream::pos_type beg = ifs.tellg();
			while (n < vertex && !ifs.eof()) {
				auto itrp = paramidx.begin();
				float f;
				double d;
				unsigned char col;
				int cnt = 0;
				for (auto itr : props) {
					switch (itr) {
					case PROP_XF:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&f, sizeof(float));
						verteces[n * 3] = f;
						cnt += 4;
						itrp++;
						break;
					case PROP_YF:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&f, sizeof(float));
						verteces[n * 3 + 1] = f;
						cnt += 4;
						itrp++;
						break;
					case PROP_ZF:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&f, sizeof(float));
						verteces[n * 3 + 2] = f;
						cnt += 4;
						itrp++;
						break;
					case PROP_XD:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&d, sizeof(double));
						verteces[n * 3] = d;
						cnt += 8;
						itrp++;
						break;
					case PROP_YD:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&d, sizeof(double));
						verteces[n * 3 + 1] = d;
						cnt += 8;
						itrp++;
						break;
					case PROP_ZD:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&d, sizeof(double));
						verteces[n * 3 + 2] = d;
						cnt += 8;
						itrp++;
						break;
					case PROP_INTENSITY:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&f, sizeof(float));
						reflectance[n] = f;
						cnt += 4;
						itrp++;
						break;
					case PROP_R:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&col, sizeof(unsigned char));
						rgba[n * 4] = col;
						cnt += 1;
						itrp++;
						break;
					case PROP_G:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&col, sizeof(unsigned char));
						rgba[n * 4 + 1] = col;
						cnt += 1;
						itrp++;
						break;
					case PROP_B:
						if (cnt != (*itrp)) {
							ifs.seekg((*itrp) - cnt, std::ios_base::cur);
							cnt = (*itrp);
						}
						ifs.read((char *)&col, sizeof(unsigned char));
						rgba[n * 4 + 2] = col;
						cnt += 1;
						itrp++;
						break;
					default:;
					}



				}
				if (paranum - cnt > 0)ifs.seekg((paranum - cnt), std::ios_base::cur);
				n++;
			}
			int facec = 0;
			while (facec < face && !ifs.eof()) {
				unsigned char i;
				int id1, id2, id3;
				ifs.read((char *)&i, sizeof(unsigned char));
				ifs.read((char *)&id1, sizeof(int));
				ifs.read((char *)&id2, sizeof(int));
				ifs.read((char *)&id3, sizeof(int));

				//	cout << i <<std::endl << id1 << id2 << id3 << std::endl;

				faces[facec * 3] = id1;
				faces[facec * 3 + 1] = id2;
				faces[facec * 3 + 2] = id3;

				facec++;

				if (ifs.eof())
					id2 = 0;
			}
			facenum = facec;
		}
		if (headString(format, "ascii")) {
			return false;
			while (n < vertex && !ifs.eof()) {
				float f[5];
				getline(ifs, line);
				int i;
				for (i = 0; i < 5; i++) {
					if (i != 4)f[i] = stof(line.substr(0, line.find_first_of(" ")));
					else f[i] = stof(line);
					line.erase(0, line.find_first_of(" ") + 1);
				}
				verteces[n * 3] = f[xi];
				verteces[n * 3 + 1] = f[yi];
				verteces[n * 3 + 2] = f[zi];
				n++;
			}
		}
		//	vertPointDataSize = paranum;
		vertexnum = n;

		ifs.close();
		return true;
	}


	void plyObject::writePlyFile(std::string fileName) {
		std::ofstream ofs(fileName, std::ios::out | std::ios::binary);
		ofs << "ply" << std::endl;
		ofs << "format binary_little_endian 1.0" << std::endl;
		ofs << "element vertex " << vertexnum << std::endl;
		ofs << "property float x" << std::endl;
		ofs << "property float y" << std::endl;
		ofs << "property float z" << std::endl;
		ofs << "property float confidence" << std::endl;
		if (bG)ofs << "property float intensity" << std::endl;
		if (bC) {
			ofs << "property uchar red" << std::endl;
			ofs << "property uchar green" << std::endl;
			ofs << "property uchar blue" << std::endl;
			ofs << "property uchar alpha" << std::endl;
		}
		ofs << "element face " << facenum << std::endl;
		ofs << "property list uchar int vertex_index" << std::endl;
		ofs << "end_header" << std::endl;
		int time;
		int i;
		for (i = 0; i < vertexnum; i++) {

			float fa[5];
			fa[0] = verteces[i * 3];
			fa[1] = verteces[i * 3 + 1];
			fa[2] = verteces[i * 3 + 2];
			fa[3] = 1.0f;
			for (time = 0; time < 4; time++) {
				ofs.write((char *)&fa[time], sizeof(float));
			}
			if (bG) {
				fa[4] = reflectance[i];
				ofs.write((char *)&fa[4], sizeof(float));
			}
			if (bC) {
				ofs.write((char *)&rgba[i * 4], sizeof(char));
				ofs.write((char *)&rgba[i * 4 + 1], sizeof(char));
				ofs.write((char *)&rgba[i * 4 + 2], sizeof(char));
				ofs.write((char *)&rgba[i * 4 + 3], sizeof(char));
			}
		}
		unsigned char ftri = 3;
		for (i = 0; i < facenum; i++) {
			ofs.write((char *)&ftri, sizeof(char));
			ofs.write((char *)&faces[i * 3], sizeof(int));
			ofs.write((char *)&faces[i * 3 + 1], sizeof(int));
			ofs.write((char *)&faces[i * 3 + 2], sizeof(int));

		}
		ofs.close();
	}




	void plyObject::transform(Eigen::Matrix4d m) {
		for (int i = 0; i < vertexnum; i++) {
			Eigen::Vector4d v;
			v << verteces[i * 3], verteces[i * 3 + 1], verteces[i * 3 + 2], 1;
			v = m * v;
			verteces[i * 3] = v(0);
			verteces[i * 3 + 1] = v(1);
			verteces[i * 3 + 2] = v(2);
		}
		GlobalPose = m * GlobalPose;
	};


	void plyObject::writePlyFileRGBForMeshlab(std::string fileName) {
		std::ofstream ofs(fileName, std::ios::out | std::ios::binary);
		ofs << "ply" << std::endl;
		ofs << "format binary_little_endian 1.0" << std::endl;
		ofs << "element vertex " << vertexnum << std::endl;
		ofs << "property float x" << std::endl;
		ofs << "property float y" << std::endl;
		ofs << "property float z" << std::endl;
		ofs << "property uchar red" << std::endl;
		ofs << "property uchar green" << std::endl;
		ofs << "property uchar blue" << std::endl;
		ofs << "element face " << facenum << std::endl;
		ofs << "property list uchar int vertex_index" << std::endl;
		ofs << "end_header" << std::endl;
		int time;
		int i;
		for (i = 0; i < vertexnum; i++) {

			float fa[5];
			fa[0] = verteces[i * 3];
			fa[1] = verteces[i * 3 + 1];
			fa[2] = verteces[i * 3 + 2];
			//fa[3] = 1.0f;
			for (time = 0; time < 3; time++) {
				ofs.write((char *)&fa[time], sizeof(float));
			}
			ofs.write((char *)&rgba[i * 4], sizeof(char));
			ofs.write((char *)&rgba[i * 4 + 1], sizeof(char));
			ofs.write((char *)&rgba[i * 4 + 2], sizeof(char));
		}
		unsigned char ftri = 3;
		for (i = 0; i < facenum; i++) {
			ofs.write((char *)&ftri, sizeof(char));
			ofs.write((char *)&faces[i * 3], sizeof(int));
			ofs.write((char *)&faces[i * 3 + 1], sizeof(int));
			ofs.write((char *)&faces[i * 3 + 2], sizeof(int));

		}
		ofs.close();
	}

	void plyObject::release() {
		if (!bRead)return;
		free(verteces);
		free(norm);
		free(reflectance);
		bRead = false;

	}

	void plyObject::computeNorm() {
		if (norm != NULL) {
			free(norm);
		}
		norm = (float*)malloc(sizeof(float)*getVertexNumber() * 3);
		memset(norm, 0, sizeof(float)*getVertexNumber() * 3);
		float* vec = getVertecesPointer();
		for (int i = 0; i < getFaceNumber(); i++) {


			Eigen::Vector3d p1, p2, p3;
			p1(0) = vec[faces[i * 3] * 3];			p1(1) = vec[faces[i * 3] * 3 + 1];			p1(2) = vec[faces[i * 3] * 3 + 2];
			p2(0) = vec[faces[i * 3 + 1] * 3];			p2(1) = vec[faces[i * 3 + 1] * 3 + 1];			p2(2) = vec[faces[i * 3 + 1] * 3 + 2];
			p3(0) = vec[faces[i * 3 + 2] * 3];			p3(1) = vec[faces[i * 3 + 2] * 3 + 1];			p3(2) = vec[faces[i * 3 + 2] * 3 + 2];

			Eigen::Vector3d nc = getNorm(p1, p2, p3);
			norm[faces[i * 3] * 3] += nc(0); norm[faces[i * 3] * 3 + 1] += nc(1); norm[faces[i * 3] * 3 + 2] += nc(2);
			norm[faces[i * 3 + 1] * 3] += nc(0); norm[faces[i * 3 + 1] * 3 + 1] += nc(1); norm[faces[i * 3 + 1] * 3 + 2] += nc(2);
			norm[faces[i * 3 + 2] * 3] += nc(0); norm[faces[i * 3 + 2] * 3 + 1] += nc(1); norm[faces[i * 3 + 2] * 3 + 2] += nc(2);

		}

	}





	void plyObject::removeNoMeshPoints() {
		bool* exists = (bool*)malloc(sizeof(bool)*vertexnum);
		memset(exists, 0, sizeof(bool)*vertexnum);
		for (int i = 0; i < facenum; i++) {
			exists[faces[i * 3]] = true;
			exists[faces[i * 3 + 1]] = true;
			exists[faces[i * 3 + 2]] = true;
		}
		std::vector<int> inlier;
		for (int i = 0; i < vertexnum; i++) {
			if (exists[i])inlier.push_back(i);
		}
		float* newv = (float*)malloc(sizeof(float)*inlier.size() * 3);
		float* newRef = (float*)malloc(sizeof(float)*inlier.size());
		int cnt = 0;
		IDXMAP idxConv;
		for (auto in_idx : inlier) {
			newv[cnt * 3] = verteces[in_idx * 3];
			newv[cnt * 3 + 1] = verteces[in_idx * 3 + 1];
			newv[cnt * 3 + 2] = verteces[in_idx * 3 + 2];
			newRef[cnt] = reflectance[in_idx];
			idxConv.insert(IDXMAP::value_type::pair(in_idx, cnt));
			cnt++;
		}
		for (int i = 0; i < facenum; i++) {
			faces[i * 3] = idxConv.at(faces[i * 3]);
			faces[i * 3 + 1] = idxConv.at(faces[i * 3 + 1]);
			faces[i * 3 + 2] = idxConv.at(faces[i * 3 + 2]);
		}



		float* temp = verteces;
		verteces = newv;
		reflectance = newRef;
		//free(temp);

		vertexnum = inlier.size();

	}

	void plyObject::removeZeroPoints() {
		bool* exists = (bool*)malloc(sizeof(bool)*vertexnum);
		memset(exists, 0, sizeof(bool)*vertexnum);
		std::vector<int> inlier;
		for (int i = 0; i < vertexnum; i++) {
			if (verteces[i * 3] != 0 || verteces[i * 3 + 1] != 0 || verteces[i * 3 + 2] != 0)inlier.push_back(i);
		}
		float* newv = (float*)malloc(sizeof(float)*inlier.size() * 3);
		float* newRef = (float*)malloc(sizeof(float)*inlier.size());
		int cnt = 0;
		IDXMAP idxConv;
		for (auto in_idx : inlier) {
			newv[cnt * 3] = verteces[in_idx * 3];
			newv[cnt * 3 + 1] = verteces[in_idx * 3 + 1];
			newv[cnt * 3 + 2] = verteces[in_idx * 3 + 2];
			newRef[cnt] = reflectance[in_idx];
			idxConv.insert(IDXMAP::value_type::pair(in_idx, cnt));
			cnt++;
		}
		for (int i = 0; i < facenum; i++) {
			faces[i * 3] = idxConv.at(faces[i * 3]);
			faces[i * 3 + 1] = idxConv.at(faces[i * 3 + 1]);
			faces[i * 3 + 2] = idxConv.at(faces[i * 3 + 2]);
		}

		float* temp = verteces;
		verteces = newv;
		reflectance = newRef;
		//free(temp);
		vertexnum = inlier.size();

	}
}
