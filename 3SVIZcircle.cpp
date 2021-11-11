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
int main(int argc, char** argv)
{
    string f = "false";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ center;
    string path1;

    if(argc == 2)
    {
        string path = argv[1];
        path1 = "/home/slishy/Code/PCD/hanjie/" + path;
    }
   else
   {
       path1 = "/home/slishy/Code/PCD/hanjie/gong.pcd";
   }
    pcl::io::loadPCDFile(path1,*cloud_in);
   
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_in->points[i].x = cloud_in->points[i].x * 0.001;
        cloud_in->points[i].y = cloud_in->points[i].y * 0.001;
        cloud_in->points[i].z = cloud_in->points[i].z * 0.001;
    }
     *cloud_out = *cloud_in;
  
    /*PointProcess pp;
    pp.SetK(20);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(cloud_in);*/
    pcl::PointXYZ pointmin,pointmax;
    pcl::getMinMax3D(*cloud_in,pointmin,pointmax);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(pointmin.z,pointmin.z + 0.01);    
    //pass.filter(*cloud_in); 
    pcl::io::savePCDFile("1.pcd",*cloud_in);
   // pcl::io::savePCDFile("/home/slishy/Code/PCD/circle/5.pcd",*cloud_in_);     
    Planefitting pf;
    pf.SetDistanceThreshold(0.002);
    pf.extractbynormal(cloud_in,cloud);
    //pf.extract(cloud_in,cloud,"false");
    //pcl::io::savePCDFile("/home/slishy/Code/PCD/circle/2.pcd",*cloud_out);
    Computepointspose cp;
    cp.GetCircleCenter(cloud,center);
    //cp.GetCenter(cloud_in,center);
    float r = center.x + 0.055;
    for (size_t i = 0; i < 1; i++)
    {
        cout << center.x << ";" << center.y<< ";" <<center.z<<endl;
    }
    Pointviewer pv;
    pv.simpleVisN(cloud_out,cloud,center);
}