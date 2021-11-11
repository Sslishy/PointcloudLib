#include "CollisionDetection.h"
float v = 0.001;
CollisionDetection::CollisionDetection()
{
	m_Toolwidth = 0;
	m_Dep = 0;
}

void CollisionDetection::ReaderConfigFile(string const path)
{
	cv::FileStorage fs(path, FileStorage::READ);
	fs["toolwidth"] >> m_Toolwidth;
	fs["dep"] >> m_Dep;
	Point3f cvp1;
	Point3f cvp2;
	Point3f cvp3;
	Point3f cvp4;
	fs["p1"] >> cvp1;
	fs["p2"] >> cvp2;
	fs["p3"] >> cvp3;
	fs["p4"] >> cvp4;
	m_Toolwidth = m_Toolwidth * v;
	m_Dep = m_Dep * v;
	p0.x = cvp1.x * v; p0.y = cvp1.y * v; p0.z = cvp1.z * v;
	p1.x = cvp2.x * v; p1.y = cvp2.y * v; p1.z = cvp2.z * v;
	p2.x = cvp3.x * v; p2.y = cvp3.y * v; p2.z = cvp3.z * v;
	p3.x = cvp4.x * v; p3.y = cvp4.y * v; p3.z = cvp4.z * v;
	p4.x = p0.x; p4.y = p0.y; p4.z = p0.z + m_Dep;
	p5.x = p1.x; p5.y = p1.y; p5.z = p1.z + m_Dep;
	p6.x = p2.x; p6.y = p2.y; p6.z = p2.z + m_Dep;
	p7.x = p3.x; p7.y = p3.y; p7.z = p3.z + m_Dep;
}
void CollisionDetection::CollisionDetection_(const vector<pcl::PointXYZ> center, const vector<pcl::Normal> N, vector<bool>& Collision)
{
	vector<pcl::Normal> Normalpoint;
	vector<pcl::PointXYZ> NormalpointXYZ;
	Normalpoint.resize(center.size());
	Collision.resize(center.size());
	NormalpointXYZ.resize(center.size());
	vector<double> DistanceOfPointToLine_;
	DistanceOfPointToLine_.resize(4);
	int r = 1000;
	//获取轮廓
	Mat src1 = Mat::zeros(Size(1500, 1500), CV_8UC1);
	Mat src = Mat::zeros(Size(1500, 1500), CV_8UC1);
	vector<Point2d> vert(4);
	vert[0] = Point2d(p0.x * 1000 + 1000, p0.y * 1000 + 1000);
	vert[1] = Point2d(p1.x * 1000 + 1000, p1.y * 1000 + 1000);
	vert[2] = Point2d(p2.x * 1000 + 1000, p2.y * 1000 + 1000);
	vert[3] = Point2d(p3.x * 1000 + 1000, p3.y * 1000 + 1000);
	for (size_t i = 0; i < vert.size(); i++)
	{
		line(src, vert[i], vert[(i + 1) % 4], Scalar(255), 3, 8, 0);
	}
	vector<vector<Point>> contours; vector<Vec4i> hierarchy;
	findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	drawContours(src1, contours, -1, (255, 255, 255), 5);
	//获取料框表面的法线点并计算是否在轮廓内
	for (size_t i = 0; i < center.size(); i++)
	{
		double len = abs((p0.z + p1.z + p2.z + p3.z) / 4 - center[i].z) / abs(N[i].normal_z);
		Normalpoint[i].normal_x = N[i].normal_x * len;
		Normalpoint[i].normal_y = N[i].normal_y * len;
		Normalpoint[i].normal_z = N[i].normal_z * len;
		NormalpointXYZ[i].x = center[i].x + Normalpoint[i].normal_x;
		NormalpointXYZ[i].y = center[i].y + Normalpoint[i].normal_y;
		NormalpointXYZ[i].z = center[i].z + Normalpoint[i].normal_z;
		//返回 -1、0、1三个固定值。若返回值为+1，表示点在多边形内部，返回值为-1，表示在多边形外部，返回值为0，表示在多边形上
		double distance = pointPolygonTest(contours[0], Point2d(NormalpointXYZ[i].x * 1000 + 1000, NormalpointXYZ[i].y * 1000 + 1000), false);
		circle(src, Point2d(NormalpointXYZ[i].x * 1000 + 1000, NormalpointXYZ[i].y * 1000 + 1000), 15, Scalar(255, 0, 0));
		if (distance < 0 || distance == 0)
		{
			Collision[i] = false;
		}
		else
		{
			//计算点到直线的距离
			DistanceOfPointToLine_[0] = DistanceOfPointToLine(p0, p1, NormalpointXYZ[i]);
			DistanceOfPointToLine_[1] = DistanceOfPointToLine(p1, p2, NormalpointXYZ[i]);
			DistanceOfPointToLine_[2] = DistanceOfPointToLine(p2, p3, NormalpointXYZ[i]);
			DistanceOfPointToLine_[3] = DistanceOfPointToLine(p3, p0, NormalpointXYZ[i]);
			if (DistanceOfPointToLine_[0] > m_Toolwidth&& DistanceOfPointToLine_[1] > m_Toolwidth&& DistanceOfPointToLine_[2] > m_Toolwidth&& DistanceOfPointToLine_[3] > m_Toolwidth)
			{
				Collision[i] = true;
			}
			else
			{
				Collision[i] = false;
			}
		}
	}
	
}
CollisionDetection::~CollisionDetection()
{

}