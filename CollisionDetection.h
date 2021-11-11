#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include<iostream>
using namespace std;
using namespace cv;
class CollisionDetection
{
public:
	CollisionDetection();
	
	inline void const SetBoxDepth(float const BoxDep)
	{
		this->m_Dep = BoxDep;
	}
	inline void const SetToolwidth(float const Toolwidth)
	{
		this->m_Toolwidth = Toolwidth;
	}
	inline double DistanceOfPointToLine(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ s)
	{
		double ab = sqrt(pow((a.x - b.x), 2.0) + pow((a.y - b.y), 2.0) + pow((a.z - b.z), 2.0));
		double as = sqrt(pow((a.x - s.x), 2.0) + pow((a.y - s.y), 2.0) + pow((a.z - s.z), 2.0));
		double bs = sqrt(pow((s.x - b.x), 2.0) + pow((s.y - b.y), 2.0) + pow((s.z - b.z), 2.0));
		double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab * as);
		double sin_A = sqrt(1 - pow(cos_A, 2.0));
		return as * sin_A;
	}
	void ReaderConfigFile(string const path);
	void CollisionDetection_(const vector<pcl::PointXYZ> center, const vector<pcl::Normal> N, vector<bool>& Collision);
	~CollisionDetection();
private:
	float m_Dep;
	float m_Toolwidth;
	pcl::PointXYZ p0;
	pcl::PointXYZ p1;
	pcl::PointXYZ p2;
	pcl::PointXYZ p3;
	pcl::PointXYZ p4;
	pcl::PointXYZ p5;
	pcl::PointXYZ p6;
	pcl::PointXYZ p7;
};

