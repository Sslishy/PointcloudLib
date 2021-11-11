#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include<Eigen/StdVector>
using namespace cv;
using namespace std;
class Pointviewer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	void ReaderConfigFile(string const path)
	{
		float v = 0.001;
		cv::FileStorage fs(path, FileStorage::READ);
		fs["dep"] >> m_Dep;
		Point3f cvp1;
		Point3f cvp2;
		Point3f cvp3;
		Point3f cvp4;
		fs["p1"] >> cvp1;
		fs["p2"] >> cvp2;
		fs["p3"] >> cvp3;
		fs["p4"] >> cvp4;
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
	Pointviewer();
	__attribute__((force_align_arg_pointer)) pcl::visualization::PCLVisualizer::Ptr simpleVisN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const vector<pcl::PointXYZ> center, const vector<pcl::Normal> N, const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, const vector<pcl::Normal> Normal_X, const vector<pcl::Normal> Normal_Y, const vector<cv::Vec3f> RPYList);
	pcl::visualization::PCLVisualizer::Ptr simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::visualization::PCLVisualizer::Ptr simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1);
	pcl::visualization::PCLVisualizer::Ptr simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointXYZ center);
	pcl::visualization::PCLVisualizer::Ptr simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr center,vector<cv::Mat> RPY);
	pcl::visualization::PCLVisualizer::Ptr simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointXYZ center,cv::Mat RPY);
	pcl::visualization::PCLVisualizer::Ptr simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointXYZ center,Eigen::Vector3d rx,Eigen::Vector3d ry ,Eigen::Vector3d rz);
private:
	cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f theta)
	{
		theta /= 180 / CV_PI;
		// Calculate rotation about x axis
		Mat R_x = (Mat_<double>(3, 3) <<
			1, 0, 0,
			0, cos(theta[0]), -sin(theta[0]),
			0, sin(theta[0]), cos(theta[0])
			);

		// Calculate rotation about y axis
		Mat R_y = (Mat_<double>(3, 3) <<
			cos(theta[1]), 0, sin(theta[1]),
			0, 1, 0,
			-sin(theta[1]), 0, cos(theta[1])
			);

		// Calculate rotation about z axis
		Mat R_z = (Mat_<double>(3, 3) <<
			cos(theta[2]), -sin(theta[2]), 0,
			sin(theta[2]), cos(theta[2]), 0,
			0, 0, 1);

		// Combined rotation matrix
		Mat R = R_x * R_y * R_z;

		return R;
	};
	pcl::PointXYZ p0;
	pcl::PointXYZ p1;
	pcl::PointXYZ p2;
	pcl::PointXYZ p3;
	pcl::PointXYZ p4;
	pcl::PointXYZ p5;
	pcl::PointXYZ p6;
	pcl::PointXYZ p7;
	float m_Dep;
};
Pointviewer::Pointviewer()
{
	p0.x = 0;
	p0.y = 0;
	p0.z = 0;
	p0=p1=p2=p3=p4=p5=p6=p7;
}
pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointXYZ center,Eigen::Vector3d rx,Eigen::Vector3d ry ,Eigen::Vector3d rz)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointXYZ rx1,ry1,rz1;
	rx1.x = rx[0] + center.x;
	rx1.y = rx[1] + center.y;
	rx1.z = rx[2] + center.z;
	ry1.x = ry[0] + center.x;
	ry1.y = ry[1] + center.y;
	ry1.z = ry[2] + center.z;
	rz1.x = rz[0] + center.x;
	rz1.y = rz[1] + center.y;
	rz1.z = rz[2] + center.z;
	viewer->addLine(rx1,center,220,20,60,"line");
	viewer->addLine(ry1,center,127,255,212,"line1");
	viewer->addLine(rz1,center,0,0,255,"line2");
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud,0.0,0.0,255.0),"cloud");
	viewer->addPointCloud(cloud1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud1,0.0,255.0,0.0),"cloud1");
	viewer->addSphere(center,0.0025,"sphere");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}
__attribute__((force_align_arg_pointer))pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointXYZ center,cv::Mat RPY)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	//Mat R = eulerAnglesToRotationMatrix(RPY);
	Mat R = RPY;
	Eigen::Matrix4f m4f_transform1;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 4) <<
			-R.at<double>(0, 0), -R.at<double>(0, 1), -R.at<double>(0, 2), center.x,
			-R.at<double>(1, 0), -R.at<double>(1, 1), -R.at<double>(1, 2), center.y,
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), center.z,
			0, 0, 0, 1
			);
	cv::cv2eigen(R1, m4f_transform1);
	Eigen::Transform<float, 3, Eigen::Affine> a3f_transform(m4f_transform1);
	viewer->addCoordinateSystem(0.05, a3f_transform, "object");
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud,0.0,0.0,255.0),"cloud");
	viewer->addPointCloud(cloud1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud1,0.0,255.0,0.0),"cloud1");
	viewer->addSphere(center,0.0025,"sphere");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}
pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const vector<pcl::PointXYZ> center, const vector<pcl::Normal> N, const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, const vector<pcl::Normal> Normal_X, const vector<pcl::Normal> Normal_Y, const vector<cv::Vec3f> RPYList)
{
	vector<pcl::PointCloud<pcl::PointXYZ>> center_;
	vector<pcl::PointCloud<pcl::Normal>> CenterNormal_;
	center_.resize(cloudlist.size());
	CenterNormal_.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
		center_[i].push_back(pcl::PointXYZ(center[i].x, center[i].y, center[i].z));
		CenterNormal_[i].push_back(pcl::Normal(N[i].normal_x, N[i].normal_y, N[i].normal_z, N[i].curvature));
	}
	vector<pcl::PointCloud<pcl::Normal>> CenterNormal_X;
	vector<pcl::PointCloud<pcl::Normal>> CenterNormal_Y;
	vector<pcl::PointXYZ> Center;
	Center.resize(center_.size());
	CenterNormal_X.resize(cloudlist.size());
	CenterNormal_Y.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
		CenterNormal_X[i].push_back(pcl::Normal(Normal_X[i].normal_x, Normal_X[i].normal_y, Normal_X[i].normal_z, Normal_X[i].curvature));
		CenterNormal_Y[i].push_back(pcl::Normal(Normal_Y[i].normal_x, Normal_Y[i].normal_y, Normal_Y[i].normal_z, Normal_Y[i].curvature));

	}
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int v1(0);
	int v2(1);
	int v3(2);
	if (cloudlist.size() == 2)
	{
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2);
		viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v3);
		viewer->addText("White: Original point cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "Original", v1);
		viewer->addText("White: Plane1 point cloud\n", 5, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "Plane1", v2);
		viewer->addText("White: Plane2 point cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "Plane2", v3);
		for (size_t i = 0; i < center_.size(); i++)
		{
			switch (i)
			{
			case 0: {
				double len = abs((p0.z + p1.z + p2.z + p3.z) / 4 - center_[i][0].z) / abs(CenterNormal_[i][0].normal_z);
				if (p0.x == 0)
				{
					len = 0;
				}
				
				viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_[i].makeShared(), 5, len, to_string(i), v2);
				//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_X[i].makeShared(), 5, 0.2, "Normal_X" + to_string(i), v2);
				//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_Y[i].makeShared(), 5, 0.2, "Normal_Y" + to_string(i), v2);
				Center[0].x = center_[0][0].x;
				Center[0].y = center_[0][0].y;
				Center[0].z = center_[0][0].z;
				viewer->addSphere(Center[0], 0.005, 255.0, 250.0, 0.0, "shape", v2);
				Mat R = eulerAnglesToRotationMatrix(RPYList[0]);
				Eigen::Matrix4f m4f_transform1;
				cv::Mat_<double> R1 = (cv::Mat_<double>(4, 4) <<
					-R.at<double>(0, 0), -R.at<double>(0, 1), -R.at<double>(0, 2), Center[0].x,
					-R.at<double>(1, 0), -R.at<double>(1, 1), -R.at<double>(1, 2), Center[0].y,
					R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), Center[0].z,
					0, 0, 0, 1
					);
				cv::cv2eigen(R1, m4f_transform1);
				Eigen::Transform<float, 3, Eigen::Affine> a3f_transform(m4f_transform1);
				viewer->addCoordinateSystem(0.05, a3f_transform, "object" + to_string(0), v2);

			}
				  break;
			case 1: {
				double len = abs((p0.z + p1.z + p2.z + p3.z) / 4 - center_[i][0].z) / abs(CenterNormal_[i][0].normal_z);
				if (p0.x == 0)
				{
					len = 0;
				}
				viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_[i].makeShared(), 5, len, to_string(i), v3);
				//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_X[i].makeShared(), 5, 0.2, "Normal_X" + to_string(i), v3);
				//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_Y[i].makeShared(), 5, 0.2, "Normal_Y" + to_string(i), v3);
				Center[1].x = center_[1][0].x;
				Center[1].y = center_[1][0].y;
				Center[1].z = center_[1][0].z;
				viewer->addSphere(Center[1], 0.005, 255.0, 250.0, 0.0, "shape1", v3);
				Mat R = eulerAnglesToRotationMatrix(RPYList[1]);
				Eigen::Matrix4f m4f_transform1;
				cv::Mat_<double> R1 = (cv::Mat_<double>(4, 4) <<
					-R.at<double>(0, 0), -R.at<double>(0, 1), -R.at<double>(0, 2), Center[1].x,
					-R.at<double>(1, 0), -R.at<double>(1, 1), -R.at<double>(1, 2), Center[1].y,
					R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), Center[1].z,
					0, 0, 0, 1
					);
				cv::cv2eigen(R1, m4f_transform1);
				Eigen::Transform<float, 3, Eigen::Affine> a3f_transform(m4f_transform1);
				viewer->addCoordinateSystem(0.05, a3f_transform, "object" + to_string(0), v3);


			}
				  break;
			}

		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(cloud, 255, 255, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloudlist[0].makeShared(), 65, 105, 225);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloudlist[1].makeShared(), 255, 0, 0);
		viewer->addPointCloud(cloud, single_color0, "cloud0", v1);
		viewer->addPointCloud(cloudlist[0].makeShared(), single_color1, "cloud1", v2);
		viewer->addPointCloud(cloudlist[1].makeShared(), single_color2, "cloud2", v3);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud0");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
		viewer->addLine(p0, p1, "line0", v2);
		viewer->addLine(p1, p2, "line1", v2);
		viewer->addLine(p2, p3, "line2", v2);
		viewer->addLine(p3, p0, "line3", v2);
		viewer->addLine(p4, p5, "line4", v2);
		viewer->addLine(p5, p6, "line5", v2);
		viewer->addLine(p6, p7, "line6", v2);
		viewer->addLine(p7, p4, "line7", v2);
		viewer->addLine(p0, p4, "line8", v2);
		viewer->addLine(p1, p5, "line9", v2);
		viewer->addLine(p2, p6, "line10", v2);
		viewer->addLine(p3, p7, "line11", v2);
		viewer->addLine(p0, p1, "line01", v3);
		viewer->addLine(p1, p2, "line1.1", v3);
		viewer->addLine(p2, p3, "line21", v3);
		viewer->addLine(p3, p0, "line31", v3);
		viewer->addLine(p4, p5, "line41", v3);
		viewer->addLine(p5, p6, "line51", v3);
		viewer->addLine(p6, p7, "line61", v3);
		viewer->addLine(p7, p4, "line71", v3);
		viewer->addLine(p0, p4, "line81", v3);
		viewer->addLine(p1, p5, "line91", v3);
		viewer->addLine(p2, p6, "line101", v3);
		viewer->addLine(p3, p7, "line111", v3);
		viewer->addCoordinateSystem(1, "glod", v1);
		viewer->addCoordinateSystem(1, "glod2", v2);
		viewer->addCoordinateSystem(1, "glod3", v3);

	}
	else
	{
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->addText("White: Original point cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
		viewer->addText("White: After point cloud\n", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
		for (size_t i = 0; i < center_.size(); i++)
		{
			//double len = abs(dep - abs(abs((p4.z+p5.z+p6.z+p7.z)/4) - abs(center_[i][0].z)));

			double len = abs((p0.z + p1.z + p2.z + p3.z) / 4 - center_[i][0].z) / abs(CenterNormal_[i][0].normal_z);
			if (p0.x == 0)
				{
					len = 0;
				}
			viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_[i].makeShared(), 5, len, to_string(i), v2);
			//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_X[i].makeShared(), 5, 0.2, "Normal_X"+to_string(i), v2);
			//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_[i].makeShared(), CenterNormal_Y[i].makeShared(), 5, 0.2, "Normal_Y"+to_string(i), v2);

		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloudlist[0].makeShared(), 255, 255, 255);
		viewer->addPointCloud(cloud, single_color, "cloud", v1);
		viewer->addPointCloud(cloudlist[0].makeShared(), single_color1, "cloud2", v2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
		vector<pcl::PointXYZ> Center;
		Center.resize(center_.size());
		Center[0].x = center_[0][0].x;
		Center[0].y = center_[0][0].y;
		Center[0].z = center_[0][0].z;
		viewer->addSphere(Center[0], 0.005, 255.0, 250.0, 0.0, "shape" + to_string(0), v2);
		Mat R = eulerAnglesToRotationMatrix(RPYList[0]);
		Eigen::Matrix4f m4f_transform1;
		cv::Mat_<double> R1 = (cv::Mat_<double>(4, 4) <<
			-R.at<double>(0, 0), -R.at<double>(0, 1), -R.at<double>(0, 2), Center[0].x,
			-R.at<double>(1, 0), -R.at<double>(1, 1), -R.at<double>(1, 2), Center[0].y,
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), Center[0].z,
			0, 0, 0, 1
			);
		cv::cv2eigen(R1, m4f_transform1);
		Eigen::Transform<float, 3, Eigen::Affine> a3f_transform(m4f_transform1);
		viewer->addCoordinateSystem(0.05, a3f_transform, "object" + to_string(0), v2);
		viewer->addLine(p0, p1, "line0", v2);
		viewer->addLine(p1, p2, "line1", v2);
		viewer->addLine(p2, p3, "line2", v2);
		viewer->addLine(p3, p0, "line3", v2);
		viewer->addLine(p4, p5, "line4", v2);
		viewer->addLine(p5, p6, "line5", v2);
		viewer->addLine(p6, p7, "line6", v2);
		viewer->addLine(p7, p4, "line7", v2);
		viewer->addLine(p0, p4, "line8", v2);
		viewer->addLine(p1, p5, "line9", v2);
		viewer->addLine(p2, p6, "line10", v2);
		viewer->addLine(p3, p7, "line11", v2);
		viewer->addCoordinateSystem(1, "glod", v1);
		viewer->addCoordinateSystem(1, "glod2", v2);
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}
