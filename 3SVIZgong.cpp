#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<math.h>
#include <ctime>
#include "Planefitting.h"
#include"Computepointspose.h"
#include"computeangle.h" 
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
void PointCloudto2D(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud_in, projectionTransform);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_in->points[i].x = 0;
    }
    pcl::transformPointCloud(*cloud_in, *cloud_in, projectionTransform.inverse());
}
int main(int argc, char** argv)
{
    string f = "false";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpath1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpathleft(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudpathright(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/test_0.pcd",*cloud_in);
    *cloud_o = *cloud_in;
    Planefitting pf;
    pf.SetDistanceThreshold(0.002);
    pf.extractbynormal(cloud_in,cloud_out);
    pcl::io::savePCDFile("/home/slishy/Code/PCD/test_2.pcd",*cloud_out);
   // pf.extract(cloud_in,cloud_out,f);
    PointCloudto2D(cloud_out);
    pcl::io::savePCDFile("/home/slishy/Code/PCD/test_1.pcd",*cloud_out);
    pcl::PointXYZ pointmax,pointmin,p1,p2,pointmax1,pointmin1;
    pcl::getMinMax3D(*cloud_out,pointmin,pointmax);
    float jj = pointmin.y;
    for (size_t i = 0; i < (pointmax.y - pointmin.y)/0.001; i++)
    {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_out);         
        pass.setFilterFieldName("y");      
        pass.setFilterLimits(jj,jj + 0.001);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        jj = jj + 0.001;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p1.z == cloudmid->points[j].z)
            {
                cloudpath->points.push_back(cloudmid->points[j]);
            }
        }
    }
    pcl::getMinMax3D(*cloudpath,pointmin,pointmax);
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(cloudpath);         
    pass1.setFilterFieldName("y");      
    pass1.setFilterLimits(pointmin.y + 0.01,pointmax.y - 0.01);    
    pass1.filter(*cloudpath);   
    pcl::getMinMax3D(*cloudpath,pointmin,pointmax);
    float mid = (pointmax.y - pointmin.y)/2;
    pass1.setInputCloud(cloudpath);         
    pass1.setFilterFieldName("y");      
    pass1.setFilterLimits(pointmin.y,mid);    
    pass1.filter(*cloudpathleft); 
    pass1.setInputCloud(cloudpath);         
    pass1.setFilterFieldName("y");      
    pass1.setFilterLimits(mid,pointmax.y);    
    pass1.filter(*cloudpathright); 
    pcl::getMinMax3D(*cloudpathleft,pointmin,pointmax);
    pcl::getMinMax3D(*cloudpathright,pointmin1,pointmax1);
    float left,right;
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if (pointmax.z == cloudpath->points[i].z)
        {
            left = cloudpath->points[i].y;
        }
        if (pointmax1.z == cloudpath->points[i].z)
        {
            right = cloudpath->points[i].y;
        }        
    }
    pass1.setInputCloud(cloudpath);         
    pass1.setFilterFieldName("y");      
    pass1.setFilterLimits(left,right);    
    pass1.filter(*cloudpath); 
    PointProcess pp;
    pp.SetLeafSize(0.008);
    pp.DownSimple(cloudpath);
    Computepointspose cp;
    vector<pcl::Normal> n;
    vector<pcl::Normal> nx;
    vector<pcl::Normal> ny;
    vector<cv::Vec3f> RPYList;
    RPYList.resize(cloudpath->points.size());
    vector<Mat> rpymat;
    rpymat.resize(cloudpath->points.size());
    n.resize(cloudpath->points.size());
    nx.resize(cloudpath->points.size());
    ny.resize(cloudpath->points.size());
    cp.SetFindNum(999);
    float max_y = cloudpath->points[0].y;
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        if (cloudpath->points[i].y > max_y)
        {
            max_y = cloudpath->points[i].y;
        }
    }
    float y[cloudpath->points.size()];
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
      
       y[i] = cloudpath->points[i].y;
    }
    sort(y,y+cloudpath->points.size());
    cloudpath1->points.resize(cloudpath->points.size());
     for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        for (size_t j = 0; j < cloudpath->points.size(); j++)
        {
          if (y[i] == cloudpath->points[j].y)
            {
                cloudpath1->points[i] = cloudpath->points[j];
            }
        }
    }
    pcl::copyPointCloud(*cloudpath1,*cloudpath);
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        cp.computePointNormal(cloud_o,cloudpath->points[i],n[i]);
    }
    for (size_t i = 0; i < cloudpath->points.size(); i++)
    {
        float angle;
        angle = Getangle(n[i]);
        if(angle > 30)
        {
            cp.SetfixedAngle(15,n[i]);
        }
    }
    computeangle ca;
    ca.ComputeDirection(n,nx,ny);
    ca.computeRPY(n,nx,ny,RPYList);
    for (size_t i = 0; i < RPYList.size(); i++)
    {
      rpymat[i] = ca.eulerAnglesToRotationMatrix(RPYList[i]);
    }
    Pointviewer pv;
    pv.simpleVisN(cloud_o,cloudpath,rpymat);
}
