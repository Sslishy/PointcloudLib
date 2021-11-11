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
using namespace std::chrono_literals;
int main(int argc, char **argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gold(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gold_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/2/1.pcd",*cloud1_);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/1/1.pcd",*cloud_gold_);
    cout << "1" <<endl;
    pcl::PointXYZ c1,c2;
    pcl::getMinMax3D(*cloud1_,c1,c2);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud1_);         
    pass.setFilterFieldName("z");      
     pass.setFilterLimits(1.10,1.20);  
    pass.filter(*cloud1_); 
    pcl::getMinMax3D(*cloud_gold_,c1,c2);
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(cloud_gold_);         
    pass1.setFilterFieldName("z");      
    pass1.setFilterLimits(1.10,1.20); 
    pass1.filter(*cloud_gold_); 
    PointProcess pp;
    pp.SetK(50);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud1_);
    pp.Removepoint(cloud_gold_);
    
    Planefitting pf;
    pf.SetDistanceThreshold(0.003);
    pf.extractbynormal(cloud1_,cloud1);
    pf.extractbynormal(cloud_gold_,cloud_gold);
    pcl::PointXYZ pointmax,pointmin ,p1 ,p2;
    pcl::getMinMax3D(*cloud1,pointmin,pointmax);
    vector<pcl::PointXYZ> index;
    index.resize(cloud1->points.size());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid1(new pcl::PointCloud<pcl::PointXYZ>);
  
  float jj = pointmin.x +0.3f;
    for (size_t i = 0; i < (pointmax.x - pointmin.x)/0.001; i++)
    {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("x");      
        pass.setFilterLimits(jj,jj + 0.001);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        jj = jj + 0.001;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p2.y == cloudmid->points[j].y)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }
    float k = pointmin.y;
    for (size_t i = 0; i < (pointmax.y - pointmin.y)/0.001; i++)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("y");      
        pass.setFilterLimits(k,k + 0.001);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        k = k + 0.001;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p2.x == cloudmid->points[j].x)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }
    pcl::getMinMax3D(*cloud_gold,pointmin,pointmax);
    vector<pcl::PointXYZ> index_gold;
    index_gold.resize(cloud_gold->points.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid_gold(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid1_gold(new pcl::PointCloud<pcl::PointXYZ>);
   jj = pointmin.x +0.3f;
    for (size_t i = 0; i < (pointmax.x - pointmin.x)/0.001; i++)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_gold);         
        pass.setFilterFieldName("x");      
        pass.setFilterLimits(jj,jj + 0.001);    
        pass.filter(*cloudmid_gold); 
        pcl::getMinMax3D(*cloudmid_gold,p1,p2);
        index_gold.push_back(p2);
        jj = jj + 0.001;
         for (size_t j = 0; j <cloudmid_gold->points.size(); j++)
        {
           if (p2.y == cloudmid_gold->points[j].y)
            {
                cloudmid1_gold->points.push_back(cloudmid_gold->points[j]);
            }
        }
    }
     k = pointmin.y;
    for (size_t i = 0; i < (pointmax.y - pointmin.y)/0.001; i++)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_gold);         
        pass.setFilterFieldName("y");      
        pass.setFilterLimits(k,k + 0.001);    
        pass.filter(*cloudmid_gold); 
        pcl::getMinMax3D(*cloudmid_gold,p1,p2);
        index_gold.push_back(p2);
        k = k + 0.001;
         for (size_t j = 0; j <cloudmid_gold->points.size(); j++)
        {
           if (p2.x == cloudmid_gold->points[j].x)
            {
                cloudmid1_gold->points.push_back(cloudmid_gold->points[j]);
            }
        }
    }
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudAligment pa;
    Pointviewer pv;
    
    pp.SetK(25);
    pp.SetStddevMulThresh(0.5);
    pp.Removepoint(cloudmid1);
    pp.Removepoint(cloudmid1_gold);
    pv.simpleVisN(cloudmid1_gold,cloudmid1);
    pa.Aligment(cloudmid1_gold,cloudmid1,cloudout);
    pv.simpleVisN(cloudout,cloudmid1);
    Eigen::Matrix4f transformation;
    pa.Gettransformation(transformation);
    cout << transformation <<endl;
    //** input path PPY **//
    pcl::PointCloud<pcl::PointXYZ>::Ptr ccc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_gold,*cloud_gold,transformation);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_gold);
	icp.setInputTarget(cloud1);
	icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance(0.01);
	icp.setRANSACOutlierRejectionThreshold(0.05);
	icp.setTransformationEpsilon(1e-10);
	icp.align(*ccc);
	Eigen::Matrix4f transformation1 = icp.getFinalTransformation();
    cout << transformation1<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr goldenpath1(new pcl::PointCloud<pcl::PointXYZ>);
    goldenpath1->points.resize(6);
    goldenpath1->points[0] = pcl::PointXYZ(1.460552, -0.187736, 1.270213);
    goldenpath1->points[1] = pcl::PointXYZ(1.460554, -0.143083, 1.270213);
    goldenpath1->points[2] = pcl::PointXYZ(1.443258, -0.143084, 1.270215);
    goldenpath1->points[3] = pcl::PointXYZ(1.443258, -0.143084, 1.31029);
    goldenpath1->points[4] = pcl::PointXYZ(1.461329, -0.143085, 1.310296);
    goldenpath1->points[5] = pcl::PointXYZ(1.461333, -0.143086, 1.512918);
    pcl::transformPointCloud(*goldenpath1,*goldenpath1,transformation);
    pcl::transformPointCloud(*goldenpath1,*goldenpath1,transformation1);
    for (size_t i = 0; i < 6; i++)
    {
        cout <<  goldenpath1->points[i] <<endl;
    }
    pcl::PointXYZ center;
    pv.simpleVisN(ccc,cloud1);
}