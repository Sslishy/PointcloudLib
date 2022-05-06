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
#include"Linefitting.h"
#include <pcl/filters/passthrough.h>
string golden_pcd;
string target_pcd;
float X,Y,Z,RX,RY,RZ,num,Ratio,DistanceThreshold,boundaryRadiusSearch,LineRatio,LineDistanceThreshold,LineNum;
int display,displayline,boundaryKSearch,planeindex,lineindex;
void LineSort(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudout)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud1);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud1, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud1, pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud1, *cloud1, projectionTransform);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
    int MaxPointZIndex;
    for (size_t i = 0; i < cloud1->points.size(); i++)
    {
        if(cloud1->points[i].z == maxPoint.z)
        {
          MaxPointZIndex = i;
        }
    }
    
	vector<int> index;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst1(new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     pcl::PointXYZ searchPoint = cloud1->points[MaxPointZIndex];
     int K = cloud1->points.size();
     kdtree.setInputCloud(cloud1);				
    std::vector<int> pointIdxSearch(K);
    std::vector<float> pointSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint,K,pointIdxSearch,pointSquaredDistance);
    for (size_t i = 0; i < pointIdxSearch.size(); i++)
    {
        cloudout->points.push_back(cloud1->points[pointIdxSearch[i]]);
    }
   
    pcl::transformPointCloud(*cloudout,*cloudout,projectionTransform.inverse());
	                                          
	
}
void readconfig()
{
  cv::FileStorage fs_read("config.yaml", cv::FileStorage::READ);
  string path;
  fs_read["golden_pcd"] >> golden_pcd;
  fs_read["target_pcd"] >> target_pcd;  
  fs_read["MinPointsNum"] >> num;
  fs_read["Ratio"] >> Ratio;
  fs_read["DistanceThreshold"] >> DistanceThreshold;
  fs_read["displayline"] >> displayline;
  fs_read["boundaryKSearch"] >> boundaryKSearch;
  fs_read["boundaryRadiusSearch"] >> boundaryRadiusSearch;
  fs_read["LineRatio"] >> LineRatio;
  fs_read["LineDistanceThreshold"] >> LineDistanceThreshold;
  fs_read["LineNum"] >> LineNum;
  fs_read["planeindex"] >> planeindex;
  fs_read["lineindex"] >> lineindex;
  fs_read["X"] >> X;  
  fs_read["Y"] >> Y;
  fs_read["Z"] >> Z;  
  fs_read["RX"] >> RX;  
  fs_read["RY"] >> RY;  
  fs_read["RZ"] >> RZ;    
  fs_read["display"] >> display;  
  fs_read.release();
}
int main(int argc, char** argv) {
    readconfig();
    Computepointspose cp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
  
    pcl::io::loadPCDFile(target_pcd,*target);
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud2;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> linecloud;
    Planefitting pf;
    pf.SetDistanceThreshold(DistanceThreshold);
    pf.SetRatio(Ratio);
    pf.SetNumofThreshold(num);
    pf.extract(target,cloud);
    
    
    Linefitting lf;
    lf.SetboundaryRadiusSearch(boundaryRadiusSearch);
    lf.SetboundaryKSearch(boundaryKSearch);
    lf.SetRatio(LineRatio);
    lf.SetMaxIterations(100);
    lf.SetDistanceThreshold(LineDistanceThreshold);
    lf.SetNumofThreshold(LineNum);
    lf.extract(cloud[planeindex],linecloud);
    pcl::PointXYZ linecenter;
    vector<pcl::PointXYZ> linecenterlist(linecloud.size());

    for (size_t i = 0; i < linecloud.size(); i++)
    {
        cp.computePickPoints(linecloud[i],linecenter);
        linecenterlist[i] = linecenter;
    }
    float minz = linecenterlist[0].z;
    int minzidx = 0;
    for (size_t i = 0; i < linecenterlist.size(); i++)
    {
        if(minz > linecenterlist[i].z)
        {
            minz = linecenterlist[i].z;
            minzidx = i;
        }
    }
  PointProcess pp;
  pp.MaxValues(linecloud[minzidx]->points[int(linecloud[minzidx]->points.size()/2)].z + 0.02);
  pp.MinValues(linecloud[minzidx]->points[int(linecloud[minzidx]->points.size()/2)].z);
  pp.limitZ(target);
  pp.SetK(300);
  pp.SetStddevMulThresh(1);
  pp.Removepoint(target);
  pf.SetDistanceThreshold(0.001);
  pf.SetRatio(0.02);
  pf.SetNumofThreshold(1000);
  pf.extract(target,cloud2);
  cloud2[0]->width = cloud2[0]->points.size();
  cloud2[1]->width = cloud2[1]->points.size();

  for (size_t i = 0; i < cloud2.size(); i++)
    {
        cout << "plane cloud " + to_string(i) + " points number:" << cloud2[i]->points.size() <<endl;
    }
  pcl::PointXYZ center;
  pcl::PointXYZ center2;
  pcl::Normal normal;
  pcl::Normal normal2;
  Eigen::Vector3f RPY;
  Eigen::Vector3f RPY2;
  cp.computePickPoints(cloud2[0],center);
  cp.computePickPoints(cloud2[1],center2);
  cp.SetFindNum(cloud2[0]->points.size());
  cp.computePointNormal(cloud2[0],center,normal);
  cp.SetFindNum(cloud2[1]->points.size());
  cp.computePointNormal(cloud2[1],center2,normal2);
  computeangle ca;
  ca.computeRPY(normal,RPY);
  ca.computeRPY(normal2,RPY2);
  cout << center <<endl;
  cout << center2 << endl;
  cout << RPY<<endl;
  cout << RPY2<<endl;
   cout <<"RX:" <<(RPY[0]-RPY2[0])/2 <<" RY:" <<(RPY[1]-RPY2[1])/2 <<" RZ:"<< (RPY[2] - RPY2[2])/2 <<endl;
    if(display == 1)
    {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(0.5);
    int R,G,B;
    for (size_t i = 0; i < cloud2.size(); i++)
    {   
        pcl::PointXYZ center;
        cp.computePickPoints(cloud2[i],center);
         R = rand() % (256) + 0;
         G = rand() % (256) + 0;
         B = rand() % (256) + 0;
        viewer->addText3D(to_string(i),center,0.01,R,G,B,to_string(i));
        viewer->addPointCloud(cloud2[i],pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud2[i],R,G,B),"cloud"+to_string(i));
    }          
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }
    if(displayline == 1)
    {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(0.5);
    int R,G,B;
    viewer->addPointCloud(target,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(target,0,200,255),"target");
    for (size_t i = 0; i < linecloud.size(); i++)
    {   
        
         R = rand() % (256) + 0;
         G = rand() % (256) + 0;
         B = rand() % (256) + 0;
        viewer->addText3D(to_string(i),linecloud[i]->points[0],0.005,R,G,B,to_string(i));
        viewer->addPointCloud(linecloud[i],pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(linecloud[i],R,G,B),"cloud"+to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,6,"cloud"+to_string(i));
        
    }          
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }

}