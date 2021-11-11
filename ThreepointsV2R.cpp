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
void save2xml(const Mat& RT)
{
	time_t tm;
	time(&tm);
	struct tm* t2 = localtime(&tm);
	char buff[1024];
	strftime(buff, sizeof(buff), "%c", t2);
	string inCailFilePath = "D:/chessborad/V2Rcalibration.xml";
	FileStorage inCailfs(inCailFilePath, FileStorage::WRITE);
	inCailfs << "calibration_time" << buff;
	inCailfs << "V2RMatrix" << RT;
	inCailfs.release();
}
cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3f>& srcPoints, const std::vector<cv::Point3f>& dstPoints)
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
		std::vector<cv::Point3f> srcPoints;
		std::vector<cv::Point3f>  dstPoints;
		float NN = 0.001;
		//src = cam  dst = robot
		srcPoints.push_back(cv::Point3f(1.52995406e+03, 7.02825714e+02, 1177));
		dstPoints.push_back(cv::Point3f(1.57594697e+03, 8.57857210e+02, 1177));

		srcPoints.push_back(cv::Point3f(1.57659572e+03, 7.84670712e+02, 1177));
		dstPoints.push_back(cv::Point3f(1.66821752e+03, 8.41776870e+02, 1177));
		
		srcPoints.push_back(cv::Point3f(1.65865487e+03, 7.38718511e+02, 1177));
		dstPoints.push_back(cv::Point3f(1.65287980e+03, 7.49399435e+02, 1177));
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
		xyz->points.resize(3);
		xyz->points[0] = pcl::PointXYZ(1.52995406e+03, 7.02825714e+02, 1177);
		xyz->points[1] = pcl::PointXYZ(1.57659572e+03, 7.84670712e+02, 1177);
		xyz->points[2] = pcl::PointXYZ(1.65865487e+03, 7.38718511e+02, 1177);

		cv::Mat RT = Get3DR_TransMatrix(srcPoints, dstPoints);
		Eigen::Matrix4f transformation;
		transformation(0,0) = RT.at<double>(0,0);
		transformation(0,1) = RT.at<double>(0,1);
		transformation(0,2) = RT.at<double>(0,2);
		transformation(0,3) = RT.at<double>(0,3);

		transformation(1,0) = RT.at<double>(1,0);
		transformation(1,1) = RT.at<double>(1,1);
		transformation(1,2) = RT.at<double>(1,2);
		transformation(1,3) = RT.at<double>(1,3);

		transformation(2,0) = RT.at<double>(2,0);
		transformation(2,1) = RT.at<double>(2,1);
		transformation(2,2) = RT.at<double>(2,2);
		transformation(2,3) = RT.at<double>(2,3);

		transformation(3,0) = 0;
		transformation(3,1) = 0;
		transformation(3,2) = 0;
		transformation(3,3) = 0; 
		cout << transformation <<endl;
		pcl::transformPointCloud(*xyz,*xyz,transformation);
		 for (size_t i = 0; i < 3; i++)
		 {
			cout << xyz->points[i]<<endl;
		 }
		 
		for (int r = 0; r < RT.rows; r++)
		{
			for (int c = 0; c < RT.cols; c++)
			{
				printf("%f, ", RT.at<double>(r, c));
			}
			printf("\n");
		}
		save2xml(RT);
		printf("**************************************\n");
		getchar();
	
}


