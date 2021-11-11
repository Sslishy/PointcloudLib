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
    pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/slishy/Code/PCD/hanjie/1.pcd",*c);
    PointProcess pp;
    pp.SetK(50);
    pp.SetStddevMulThresh(1);
    pp.Removepoint(c);
    Planefitting pf;
    pf.SetDistanceThreshold(0.001);
    pf.extractbynormal(c,c1);
    Pointviewer pv;
    pv.simpleVisN(c1);
}
