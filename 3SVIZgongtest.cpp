#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<math.h>
#include <ctime>
#include"Computepointspose.h"
#include"CollisionDetection.h"
#include"Pointviewer.h"
#include"cylinderfitting.h"
#include"PointProcess.h"
#include"PointCloudAligment.h"
#include <pcl/filters/passthrough.h>
using namespace std;
using namespace cv;
 float Getangle(pcl::Normal normal)
{
    float x,outangle;
    x = acos(abs(normal.normal_z)); 
    outangle = (180/CV_PI)*x;
    return outangle;
}
void Getleft(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpathall(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
	pcl::PointXYZ minPoint, maxPoint;
    PointProcess pp;
    pp.SetK(5);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud1);
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pp.MinValues(maxPoint.z - 0.003);
    pp.MaxValues(maxPoint.z);
    pp.limitZ(cloud1);
    
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pp.MinValues(minPoint.y);
    pp.MaxValues(maxPoint.y - 0.002);
    pp.limitY(cloud1);
    *cloudpath = *cloud1;
    pcl::getMinMax3D(*cloudpath,minPoint,maxPoint);
    pcl::PointXYZ searchPoint;
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if (maxPoint.y == cloudpath->points[i].y)
        {
            searchPoint = cloudpath->points[i];
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudpath);
    int K = cloudpath->points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
       cloudpathall->points.push_back(cloudpath->points[pointIdxNKNSearch[i]]); 
    }
     pcl::transformPointCloud(*cloudpathall, *cloud_out, projectionTransform.inverse());
}
void GetWHWithCorner(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpathall(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
    Pointviewer pv;
    
	pcl::PointXYZ minPoint, maxPoint;

    
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(cloud1);         
    pass1.setFilterFieldName("z");      
    pass1.setFilterLimits(maxPoint.z - 0.05,maxPoint.z);    
    pass1.filter(*cloudpath); 
    pcl::getMinMax3D(*cloudpath, minPoint, maxPoint);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ones(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if(cloudpath->points[i].y == minPoint.y)
        {
            ones->points.push_back(cloudpath->points[i]);
        }
    }
     pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pass1.setInputCloud(cloud1);         
    pass1.setFilterFieldName("z");      
    pass1.setFilterLimits(minPoint.z ,maxPoint.z - 0.05);    
    pass1.filter(*cloudpath1); 
    pcl::getMinMax3D(*cloudpath1, minPoint, maxPoint);
    for (size_t i = 0; i < cloudpath1->points.size(); i++)
    {
        if(cloudpath1->points[i].y == minPoint.y)
        {
            ones->points.push_back(cloudpath1->points[i]);
        }
    }
  



    PointProcess pp;
    /*pp.SetK(5);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud1);
   */
    
    if (ones->points[0].z > ones->points[1].z)
    {
      pp.MinValues(ones->points[1].z);
    pp.MaxValues(ones->points[0].z );
    pp.limitZ(cloud1);
    }
    else
    {
    pp.MinValues(ones->points[0].z);
    pp.MaxValues(ones->points[1].z );
    pp.limitZ(cloud1);
    }
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    
   // pp.MinValues(maxPoint.y + 0.005);
    //pp.MaxValues(minPoint.y - 0.005);
    //pp.limitY(cloud1);
   pv.simpleVisN(cloud1);
    pp.SetLeafSize(0.006);
    pp.DownSimple(cloud1);
     
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud1);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(maxPoint.z - 0.05,maxPoint.z);    
    pass.filter(*cloudpath); 
    pass.setInputCloud(cloud1);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(minPoint.z ,maxPoint.z - 0.05);    
    pass.filter(*cloudpath1); 
    pcl::getMinMax3D(*cloudpath,minPoint,maxPoint);
    pcl::PointXYZ searchPoint;
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if (minPoint.y == cloudpath->points[i].y)
        {
            searchPoint = cloudpath->points[i];
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudpath);
    int K = cloudpath->points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < 10; i++)
    {
       cloudpathall->points.push_back(cloudpath->points[pointIdxNKNSearch[i]]); 
    }


    kdtree.setInputCloud(cloudpath1);
    int K1 = cloudpath1->points.size();
    pcl::getMinMax3D(*cloudpath1,minPoint,maxPoint);
     for (size_t i = 0; i < cloudpath1->points.size(); i++)
    {
        if (minPoint.y == cloudpath1->points[i].y)
        {
            searchPoint = cloudpath1->points[i];
        }
    }
    pointIdxNKNSearch.resize(K1);
    pointNKNSquaredDistance.resize(K1);
    kdtree.nearestKSearch(searchPoint, K1, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < 10; i++)
    {
       cloudpathall->points.push_back(cloudpath1->points[pointIdxNKNSearch[i]]); 
    } 
    cout << ones->points[0] <<endl;
     cout << ones->points[1] <<endl;
    cloudpathall->points[0] = ones->points[0];
    cloudpathall->points[10] = ones->points[1];
    pcl::transformPointCloud(*cloudpathall, *cloud_out, projectionTransform.inverse());

    /*
	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	width = pointbox.size.width;
	Height = pointbox.size.height;
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	pcl::visualization::PCLVisualizer viewer("viewer");
	//getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(4);
	viewer.addPointCloud(cloud1,"cloud");
	viewer.addCoordinateSystem();	
	point2.x = 0;
	point2.y = rect[0].y - 100;
	point2.z = rect[0].x - 100;
	point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;
	cloudmid->points[0] = point2;
	cloudmid->points[1] = point3;
	cloudmid->points[2] = point4;
	cloudmid->points[3] = point5;
	pcl::transformPointCloud(*cloudmid,*cloudmid,projectionTransform.inverse());
	lineorgin[0] = cloudmid->points[0];
	lineorgin[1] = cloudmid->points[1];
	lineorgin[2] = cloudmid->points[2];
	lineorgin[3] = cloudmid->points[3];
	/*point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;*/
	/*viewer.addSphere(point2,0.02,"1");
	viewer.addSphere(point3,0.02,"2");
	viewer.addSphere(point4,0.02,"3");
	viewer.addSphere(point5,0.02,"4");*/
	//viewer.addLine(point2,point4,"l1");
	//viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
	
}
void leftpath(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpathall(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
    Pointviewer pv;
    
	pcl::PointXYZ minPoint, maxPoint;

    
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(cloud1);         
    pass1.setFilterFieldName("z");      
    pass1.setFilterLimits(maxPoint.z - 0.05,maxPoint.z);    
    pass1.filter(*cloudpath); 
    pcl::getMinMax3D(*cloudpath, minPoint, maxPoint);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ones(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if(cloudpath->points[i].y == minPoint.y)
        {
            ones->points.push_back(cloudpath->points[i]);
        }
    }
     pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pass1.setInputCloud(cloud1);         
    pass1.setFilterFieldName("z");      
    pass1.setFilterLimits(minPoint.z ,maxPoint.z - 0.05);    
    pass1.filter(*cloudpath1); 
    pcl::getMinMax3D(*cloudpath1, minPoint, maxPoint);
    for (size_t i = 0; i < cloudpath1->points.size(); i++)
    {
        if(cloudpath1->points[i].y == minPoint.y)
        {
            ones->points.push_back(cloudpath1->points[i]);
        }
    }
    PointProcess pp;
    /*pp.SetK(5);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud1);
   */
    if (ones->points[0].z > ones->points[1].z)
    {
      pp.MinValues(ones->points[1].z);
    pp.MaxValues(ones->points[0].z );
    pp.limitZ(cloud1);
    }
    else
    {
    pp.MinValues(ones->points[0].z);
    pp.MaxValues(ones->points[1].z );
    pp.limitZ(cloud1);
    }
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    
   // pp.MinValues(maxPoint.y + 0.005);
    //pp.MaxValues(minPoint.y - 0.005);
    //pp.limitY(cloud1);
   pv.simpleVisN(cloud1);
    pp.SetLeafSize(0.006);
    pp.DownSimple(cloud1);
     
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud1);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(maxPoint.z - 0.05,maxPoint.z);    
    pass.filter(*cloudpath); 
    pass.setInputCloud(cloud1);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(minPoint.z ,maxPoint.z - 0.05);    
    pass.filter(*cloudpath1); 
    pcl::getMinMax3D(*cloudpath,minPoint,maxPoint);
    pcl::PointXYZ searchPoint;
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if (minPoint.y == cloudpath->points[i].y)
        {
            searchPoint = cloudpath->points[i];
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudpath);
    int K = cloudpath->points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < 10; i++)
    {
       cloudpathall->points.push_back(cloudpath->points[pointIdxNKNSearch[i]]); 
    }


    kdtree.setInputCloud(cloudpath1);
    int K1 = cloudpath1->points.size();
    pcl::getMinMax3D(*cloudpath1,minPoint,maxPoint);
     for (size_t i = 0; i < cloudpath1->points.size(); i++)
    {
        if (minPoint.y == cloudpath1->points[i].y)
        {
            searchPoint = cloudpath1->points[i];
        }
    }
    pointIdxNKNSearch.resize(K1);
    pointNKNSquaredDistance.resize(K1);
    kdtree.nearestKSearch(searchPoint, K1, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < 10; i++)
    {
       cloudpathall->points.push_back(cloudpath1->points[pointIdxNKNSearch[i]]); 
    } 
    cout << ones->points[0] <<endl;
     cout << ones->points[1] <<endl;
    cloudpathall->points[0] = ones->points[0];
    cloudpathall->points[10] = ones->points[1];
    pcl::transformPointCloud(*cloudpathall, *cloud_out, projectionTransform.inverse());
}
void Getright(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpathall(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
	pcl::PointXYZ minPoint, maxPoint;
    PointProcess pp;
    pp.SetK(5);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud1);
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pp.MinValues(minPoint.z);
    pp.MaxValues(minPoint.z + 0.003);
    pp.limitZ(cloud1);
    
    pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    pp.MinValues(minPoint.y);
    pp.MaxValues(maxPoint.y - 0.002);
    pp.limitY(cloud1);
    *cloudpath = *cloud1;
    pcl::getMinMax3D(*cloudpath,minPoint,maxPoint);
    pcl::PointXYZ searchPoint;
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if (maxPoint.y == cloudpath->points[i].y)
        {
            searchPoint = cloudpath->points[i];
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudpath);
    int K = cloudpath->points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
       cloudpathall->points.push_back(cloudpath->points[pointIdxNKNSearch[i]]); 
    }
     pcl::transformPointCloud(*cloudpathall, *cloud_out, projectionTransform.inverse());
}
int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile("/home/slishy/Code/PCD/hanjie/target1.pcd",*cloud_in_);
    /* for(size_t i =0;i < cloud_in_->points.size(); i++)
   {
   	cloud_in_->points[i].x = cloud_in_->points[i].x * 0.001;
   	cloud_in_->points[i].y = cloud_in_->points[i].y * 0.001;
   	cloud_in_->points[i].z = cloud_in_->points[i].z * 0.001;
   }*/
   GetWHWithCorner(cloud_in_,cloud_out);
    for (size_t i = 0; i < 8; i++)
    {
        cout << cloud_out->points[i] <<endl;
    }
   Pointviewer pv;
   pv.simpleVisN(cloud_out);
    
} 
