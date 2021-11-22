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
void GetWHWithCorner(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float &width,float &Height,vector<pcl::PointXYZ> &lineorgin)
{
	lineorgin.resize(4);
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
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloud1, minPoint, maxPoint);
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
	viewer.addSphere(point2,0.02,"1");
	viewer.addSphere(point3,0.02,"2");
	viewer.addSphere(point4,0.02,"3");
	viewer.addSphere(point5,0.02,"4");
	//viewer.addLine(point2,point4,"l1");
	//viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
	
}
int main(int argc, char** argv)
{/*
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
    pv.simpleVisN(cloud_o,cloudpath,rpymat);*/
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile("/home/slishy/Code/PCD/hanjie/target1.pcd",*cloud_in_);
   vector<pcl::PointXYZ> o;
   float w,h;
   GetWHWithCorner(cloud_in_, w, h,o);
   pcl::PointXYZ minpoint,maxpoint,p1,p2;
   pcl::getMinMax3D(*cloud_in_,minpoint,maxpoint);
   PointProcess pp;
   pp.MinValues(maxpoint.y - 0.02); pp.MaxValues(maxpoint.y);
   pp.limitY(cloud_in_);
   Planefitting pf;
   pp.SetK(20);
   pp.SetStddevMulThresh(0.5);
   pp.Removepoint(cloud_in_);
   pf.SetDistanceThreshold(0.001);
   pf.extract(cloud_in_,cloud_in,"false");
   Pointviewer pv;
   pv.simpleVisN(cloud_in);
   pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid (new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_path(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_path2(new pcl::PointCloud<pcl::PointXYZ>); 
	normEst.setInputCloud(cloud_in); 
	normEst.setRadiusSearch(0.003); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	boundEst.setInputCloud(cloud_in); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.setKSearch(1000);
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < cloud_in->points.size(); i++) 
	{ 
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(cloud_in->points[i]); 
		} 
	} 
    pcl::getMinMax3D(*cloud_boundary,minpoint,maxpoint);
    pp.MinValues(minpoint.x + 0.004); pp.MaxValues(maxpoint.x - 0.004);
    cout << minpoint.x << " " << maxpoint.x <<endl;
    pp.limitX(cloud_boundary);
     pcl::getMinMax3D(*cloud_boundary,minpoint,maxpoint);
    pp.MinValues(minpoint.z + 0.004); pp.MaxValues(maxpoint.z - 0.004);
    pp.limitZ(cloud_boundary);
    pp.SetK(3);
    pp.SetStddevMulThresh(0.05);
    pp.Removepoint(cloud_boundary);
    pp.SetLeafSize(0.003);
    pp.DownSimple(cloud_boundary);
    pp.SetLeafSize(0.008);
    pp.DownSimple(cloud_boundary);
    pcl::getMinMax3D(*cloud_boundary,minpoint,maxpoint);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_boundary);         
    pass.setFilterFieldName("x");      
    pass.setFilterLimits(maxpoint.x - 0.05,maxpoint.x);    
    pass.filter(*cloud_mid); 
    pcl::io::savePCDFile("cloudmid.pcd",*cloud_mid);
    pass.setInputCloud(cloud_boundary);         
    pass.setFilterFieldName("x");      
    pass.setFilterLimits(minpoint.x ,maxpoint.x - 0.05);    
    pass.filter(*cloud_path); 
    pcl::io::savePCDFile("cloud_boundary.pcd",*cloud_boundary);
     pcl::io::savePCDFile("cloud_path.pcd",*cloud_path);
    pcl::getMinMax3D(*cloud_mid,minpoint,maxpoint);
     pcl::PointXYZ searchPoint;
    for (size_t i = 0; i < cloud_mid->points.size(); i++)
    {
        if (maxpoint.z == cloud_mid->points[i].z)
        {
            searchPoint = cloud_mid->points[i];
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_mid);
    int K = cloud_mid->points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < cloud_mid->points.size(); i++)
    {
       cloud_path2->points.push_back(cloud_mid->points[pointIdxNKNSearch[i]]); 
    }
    kdtree.setInputCloud(cloud_path);
    int K1 = cloud_path->points.size();
    pcl::getMinMax3D(*cloud_path,minpoint,maxpoint);
     for (size_t i = 0; i < cloud_path->points.size(); i++)
    {
        if (maxpoint.z == cloud_path->points[i].z)
        {
            searchPoint = cloud_path->points[i];
        }
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < cloud_path->points.size(); i++)
    {
       cloud_path2->points.push_back(cloud_path->points[pointIdxNKNSearch[i]]); 
    }    
    for (size_t i = 0; i < 20; i++)
    {
        cout << cloud_path2->points[i] <<endl;
    }
 
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(0.5);
	//viewer->addPointCloud(cloud_path2,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_path2,0.0,0.0,255.0),"cloud");
    viewer->addPointCloud(cloud_boundary,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_mid,255.0,255.0,0.0),"cloud1");
	for (size_t i = 0; i < cloud_path2->points.size(); i++)
    {
       viewer->addText3D(to_string(i),cloud_path2->points[i],0.001);
    }
    while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
