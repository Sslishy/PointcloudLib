#include <opencv2/opencv.hpp>
#include <iostream>
#include <iostream>
#include <thread>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include"Planefitting.h"
#include"Computepointspose.h"
#include"PointProcess.h"
#include"PointCloudAligment.h"
#include"computeangle.h"
#include"Pointviewer.h"
using namespace cv;
using namespace std;
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;
cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints)
{
	double srcSumX = 0.0f;
	double srcSumY = 0.0f;
	double srcSumZ = 0.0f;

	double dstSumX = 0.0f;
	double dstSumY = 0.0f;
	double dstSumZ = 0.0f;

	//至少三组点
	if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
	{
		return cv::Mat();
	}
	int pointsNum = srcPoints.size();
	for (int i = 0; i < pointsNum; ++i)
	{
		srcSumX += srcPoints[i].x;
		srcSumY += srcPoints[i].y;
		srcSumZ += srcPoints[i].z;

		dstSumX += dstPoints[i].x;
		dstSumY += dstPoints[i].y;
		dstSumZ += dstPoints[i].z;
	}

	cv::Point3d centerSrc, centerDst;

	centerSrc.x = double(srcSumX / pointsNum);
	centerSrc.y = double(srcSumY / pointsNum);
	centerSrc.z = double(srcSumZ / pointsNum);

	centerDst.x = double(dstSumX / pointsNum);
	centerDst.y = double(dstSumY / pointsNum);
	centerDst.z = double(dstSumZ / pointsNum);

	//Mat::Mat(int rows, int cols, int type)
	cv::Mat srcMat(3, pointsNum, CV_64FC1);
	cv::Mat dstMat(3, pointsNum, CV_64FC1);
		for (int i = 0; i < pointsNum; ++i)//N组点
		{
			//三行
			srcMat.at<double>(0, i) = srcPoints[i].x - centerSrc.x;
			srcMat.at<double>(1, i) = srcPoints[i].y - centerSrc.y;
			srcMat.at<double>(2, i) = srcPoints[i].z - centerSrc.z;

			dstMat.at<double>(0, i) = dstPoints[i].x - centerDst.x;
			dstMat.at<double>(1, i) = dstPoints[i].y - centerDst.y;
			dstMat.at<double>(2, i) = dstPoints[i].z - centerDst.z;

		}

	cv::Mat matS = srcMat * dstMat.t();

	cv::Mat matU, matW, matV;
	cv::SVDecomp(matS, matW, matU, matV);

	cv::Mat matTemp = matU * matV;
	double det = cv::determinant(matTemp);//行列式的值

	double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
	cv::Mat matM(3, 3, CV_64FC1, datM);

	cv::Mat matR = matV.t() * matM * matU.t();

	double* datR = (double*)(matR.data);
	double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
	double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
	double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);


	//生成RT齐次矩阵(4*4)
	cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
		matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
		matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
		matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
		0, 0, 0, 1
		);

	return R_T;
}
int main()
{
	vector<cv::Point3d> cam(4);
	cam[0] = Point3d(0.078209392726,0.003562615719,0.794142544270);
	cam[1] = Point3d(0.070536784828,0.103486642241,0.767597019672);
	cam[2] = Point3d(-0.096156649292,0.094384290278,0.782210350037);
	cam[3] = cam[0] + (cam[2] -cam[1]);
	
	vector<cv::Point3d> robot(4);
	robot[0] = Point3d(963.6 * 0.001 , -106.5 * 0.001 , 160.9* 0.001);
	robot[1] = Point3d(860.0 * 0.001 , -97.9 * 0.001 , 160.9* 0.001);
	robot[2] = Point3d(873.6 * 0.001 , 68.9 * 0.001 , 160.9* 0.001);
	robot[3] = robot[0] + (robot[2] -robot[1]);
	Mat matrix;
	Mat out;
	matrix = Get3DR_TransMatrix(cam, robot);
	//cv::estimateAffine3D(cam,robot,matrix,out,3.0,0.99);
	pcl::PointCloud<pcl::PointXYZ>::Ptr campose(new pcl::PointCloud<pcl::PointXYZ>);
	campose->points.resize(1);
	campose->points[0].x = 0.118590;
	campose->points[0].y = -0.026332;
	campose->points[0].z = 0.67;
	/*campose->points[0].x = 908.9 * 0.001;
	campose->points[0].y = 92.5* 0.001;
	campose->points[0].z = 542.3* 0.001;*/
	Eigen::Matrix4f matrix1;
	cv::cv2eigen(matrix,matrix1);

	computeangle ca;
	//chu shi biao ding wei zi
	Eigen::Vector3f rpy; 
	rpy = Eigen::Vector3f(-23.539 , 38.26 , 75.483);
	Eigen::Matrix3f robot2robot = ca.eulerAnglesToRotationMatrix(rpy);
	Eigen::Matrix4f robot2robot1;
	robot2robot1.block<3,3>(0,0) = robot2robot.block<3,3>(0,0);
	robot2robot1(0,3) = (789.067) *0.001;
	robot2robot1(1,3) = (49.985) *0.001;
	robot2robot1(2,3) = (614.627) *0.001;
	robot2robot1(3,3) = 1;
	// bian huan hou de wei zhi
	Eigen::Vector3f rpy1;
	rpy1 = Eigen::Vector3f(12.8 , 39.1 , 117.1);
	Eigen::Matrix3f robot2robot2 = ca.eulerAnglesToRotationMatrix(rpy1);
	Eigen::Matrix4f robot2robot3;
	robot2robot3.block<3,3>(0,0) = robot2robot2.block<3,3>(0,0);
	robot2robot3(0,3) = (908.9) *0.001;
	robot2robot3(1,3) = (92.5) *0.001;
	robot2robot3(2,3) = (542.3) *0.001;
	robot2robot3(3,3) = 1;
	matrix1(3,3) = 1;
	matrix1(3,0) = 0;
	Eigen::Matrix4f robot2posetorobot1pose;
	robot2posetorobot1pose = robot2robot3 * robot2robot1.inverse();
	cout << robot2posetorobot1pose <<endl;
	Eigen::Matrix4f camtobase;
	camtobase = robot2posetorobot1pose * matrix1;
	cout << camtobase <<endl;
	pcl::transformPointCloud(*campose,*campose,camtobase.inverse());
	//pcl::transformPointCloud(*campose,*campose,robot2posetorobot1pose.inverse());
	//pcl::transformPointCloud(*campose,*campose,robot2posetorobot1pose.inverse());
	for (size_t i = 0; i < 1; i++)
	{
		cout << campose->points[i] <<endl;
	}
}