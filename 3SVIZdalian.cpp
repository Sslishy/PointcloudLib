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
int main(int argc, char** argv) {
    string o = "o";
    string x = "x";
    string y = "y";
   if (argv[1] == x)
    {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/2/x.pcd",*cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudorg(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudorg = *cloud;
    Planefitting pf;
    pf.SetDistanceThreshold(0.005);
    pf.extract(cloud,cloud1,"false");
    pcl::PointXYZ pointmax,pointmin,pointcenter ,p1 ,p2;
    pcl::getMinMax3D(*cloud1,pointmin,pointmax);
    vector<pcl::PointXYZ> index;
    index.resize(cloud1->points.size());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid1(new pcl::PointCloud<pcl::PointXYZ>);
   cout << (pointmax.x - pointmin.x)/0.002<<endl;
   float jj = pointmin.x;
    for (size_t i = 0; i < (pointmax.x - pointmin.x)/0.002; i++)
    {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("x");      
        pass.setFilterLimits(jj,jj + 0.002);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        jj = jj + 0.002;
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
        pass.setFilterLimits(k,k + 0.002);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p1);
        k = k + 0.002;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p1.x == cloudmid->points[j].x)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }
    PointProcess pp;
    pp.SetK(30);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloudmid1);
    pp.Removepoint(cloudmid1);
    //pp.smoothxyz(cloudmid1);
    int PointCloudindex;
    Computepointspose cp;
    cp.PointCloudGetCrossPoint(cloudmid1, PointCloudindex);
   
   cout<< cloudmid1->points[PointCloudindex].x <<";" << cloudmid1->points[PointCloudindex].y <<";" << cloudmid1->points[PointCloudindex].z << endl;
    Pointviewer pv;
    pv.simpleVisN(cloudmid1,cloudorg,cloudmid1->points[PointCloudindex]);
    }
    if (argv[1] == o)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/2/o.pcd",*cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudorg(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudorg = *cloud;
    Planefitting pf;
    pf.SetDistanceThreshold(0.005);
    pf.extract(cloud,cloud1,"false");
    pcl::PointXYZ pointmax,pointmin,pointcenter ,p1 ,p2;
    pcl::getMinMax3D(*cloud1,pointmin,pointmax);
    vector<pcl::PointXYZ> index;
    index.resize(cloud1->points.size());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid1(new pcl::PointCloud<pcl::PointXYZ>);
   cout << (pointmax.x - pointmin.x)/0.002<<endl;
   float jj = pointmin.x;
    for (size_t i = 0; i < (pointmax.x - pointmin.x)/0.002; i++)
    {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("x");      
        pass.setFilterLimits(jj,jj + 0.002);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        jj = jj + 0.002;
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
        pass.setFilterLimits(k,k + 0.002);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        k = k + 0.002;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p2.x == cloudmid->points[j].x)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }
    PointProcess pp;
    pp.SetK(30);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloudmid1);
    pp.Removepoint(cloudmid1);
    //pp.smoothxyz(cloudmid1);
    int PointCloudindex;
    Computepointspose cp;
    cp.PointCloudGetCrossPoint(cloudmid1, PointCloudindex);
   cout<< cloudmid1->points[PointCloudindex].x <<";" << cloudmid1->points[PointCloudindex].y <<";" << cloudmid1->points[PointCloudindex].z << endl;
    Pointviewer pv;
    pv.simpleVisN(cloudmid1,cloudorg,cloudmid1->points[PointCloudindex]);
    }
    if (argv[1] == y)
    {
    PointProcess pp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/1/y.pcd",*cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudorg(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudorg = *cloud;
    pp.SetK(50);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud);
    Planefitting pf;
    pf.SetDistanceThreshold(0.005);
    pf.extract(cloud,cloud1,"false");
    pcl::PointXYZ pointmax,pointmin,pointcenter ,p1 ,p2;
    pcl::getMinMax3D(*cloud1,pointmin,pointmax);
    vector<pcl::PointXYZ> index;
    index.resize(cloud1->points.size());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid1(new pcl::PointCloud<pcl::PointXYZ>);
   cout << (pointmax.x - pointmin.x)/0.002<<endl;
   float jj = pointmin.x;
    for (size_t i = 0; i < (pointmax.x - pointmin.x)/0.002; i++)
    {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud1);         
        pass.setFilterFieldName("x");      
        pass.setFilterLimits(jj,jj + 0.002);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p1);
        jj = jj + 0.002;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p1.y == cloudmid->points[j].y)
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
        pass.setFilterLimits(k,k + 0.002);    
        pass.filter(*cloudmid); 
        pcl::getMinMax3D(*cloudmid,p1,p2);
        index.push_back(p2);
        k = k + 0.002;
         for (size_t j = 0; j <cloudmid->points.size(); j++)
        {
           if (p2.x == cloudmid->points[j].x)
            {
                cloudmid1->points.push_back(cloudmid->points[j]);
            }
        }
    }
   
    pp.SetK(30);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloudmid1);
    pp.Removepoint(cloudmid1);
    //pp.smoothxyz(cloudmid1);
    int PointCloudindex;
    Computepointspose cp;
    cp.PointCloudGetCrossPoint(cloudmid1, PointCloudindex);
   cout<< cloudmid1->points[PointCloudindex].x <<";" << cloudmid1->points[PointCloudindex].y <<";" << cloudmid1->points[PointCloudindex].z << endl;
    Pointviewer pv;
    pv.simpleVisN(cloudmid1,cloudorg,cloudmid1->points[PointCloudindex]);
    }
    
}