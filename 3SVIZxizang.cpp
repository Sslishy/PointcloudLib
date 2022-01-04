#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/boundary.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include"PointProcess.h"
#include"Pointviewer.h"
int main(int argc, char** argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr path(new pcl::PointCloud<pcl::PointXYZ>);
    path->points.resize(24);
    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    // 填入点云数据
    string strarguments = argv[1];
    int arguments = atoi(strarguments.c_str());
    if (arguments == 3)
    {
        pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/slishy/Code/PCD/hanjie/target2.pcd",
                               *cloud); 
    
    PointProcess pp;
    pp.SetLeafSize(0.004);
    pp.DownSimple(cloud);

    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane);
    pcl::io::savePCDFile("c_plane.pcd",*c_plane);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane1(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane1);
    pcl::io::savePCDFile("c_plane1.pcd",*c_plane1);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane2);
    pcl::io::savePCDFile("c_plane2.pcd",*c_plane2);
     extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);


     seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane3(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane3);
    pcl::io::savePCDFile("c_plane3.pcd",*c_plane3);
     extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);



    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane4(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane4);
    pcl::io::savePCDFile("c_plane4.pcd",*c_plane4);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);
    //cout << cloud->points.size() <<endl;
    vector<pcl::PointCloud<pcl::PointXYZ>> pointlist;
    pointlist.resize(5);
    
    vector<float> x_distance(5);
    x_distance[0] = abs(c_plane->points[0].x - c_plane->points[c_plane->points.size()-1].x);
    x_distance[1] = abs(c_plane1->points[0].x - c_plane1->points[c_plane1->points.size()-1].x);
    x_distance[2] = abs(c_plane2->points[0].x - c_plane2->points[c_plane2->points.size()-1].x);
    x_distance[3] = abs(c_plane3->points[0].x - c_plane3->points[c_plane3->points.size()-1].x);
    x_distance[4] = abs(c_plane4->points[0].x - c_plane4->points[c_plane4->points.size()-1].x);
    float max = x_distance[0];
    int index_= 0;
    int index = 0;
/***** 寻找中间横着的点 *****/

    for (size_t i = 0; i < x_distance.size(); i++)
    {
        if (max < x_distance[i])
        {
            max = x_distance[i];
            index_ = i;
        }
    }
    
    pcl::PointXYZ minPoint, maxPoint;
    pcl::PointXYZ searchPoint;

    if(index == 0)
    {
        pcl::getMinMax3D(*c_plane, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane);
        int K = c_plane->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        for (size_t i = 0; i < c_plane->points.size(); i++)
        {
            if(maxPoint.x == c_plane->points[i].x)
            {
                searchPoint = c_plane->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[0].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            cloud->points[i] = c_plane->points[pointIdxNKNSearch[i]];
            pointlist[0].points[i] = cloud->points[i];
        }
        
       
       
        /*path->points[4] = c_plane->points[pointIdxNKNSearch[1]];
        path->points[15] = c_plane->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
        int j = c_plane->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane->points.size() / 12);
        }*/
    }   
    if(index == 0)
    {
        pcl::getMinMax3D(*c_plane1, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane1);
        int K = c_plane1->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane1->points.size(); i++)
        {
            if(maxPoint.x == c_plane1->points[i].x)
            {
                searchPoint = c_plane1->points[i];
            }
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
        pointlist[1].resize(pointIdxNKNSearch.size());
        cloud->points.resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane1->points[pointIdxNKNSearch[i]];
            pointlist[1].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane1->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane1->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane1->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane1->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane1->points.size() / 12);
        }*/
    } 
    if(index == 0)
    {
    
       pcl::getMinMax3D(*c_plane2, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane2);
        int K = c_plane2->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane2->points.size(); i++)
        {
            if(maxPoint.x == c_plane2->points[i].x)
            {
                searchPoint = c_plane2->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[2].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane2->points[pointIdxNKNSearch[i]];
            pointlist[2].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane2->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane2->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane2->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane2->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane2->points.size() / 12);
        }*/
    } 
    if(index == 0)
    {
    
       pcl::getMinMax3D(*c_plane3, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane3);
        int K = c_plane3->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane3->points.size(); i++)
        {
            if(maxPoint.x == c_plane3->points[i].x)
            {
                searchPoint = c_plane3->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
          cloud->points.resize(pointIdxNKNSearch.size());
          pointlist[3].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane3->points[pointIdxNKNSearch[i]];
            pointlist[3].points[i] = cloud->points[i];
        }

      
       /* path->points[4] = c_plane3->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane3->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane3->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane3->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane3->points.size() / 12);
        }*/
    }
    if(index == 0)
    {
    
        pcl::getMinMax3D(*c_plane4, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane4);
        int K = c_plane4->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane4->points.size(); i++)
        {
            if(maxPoint.x == c_plane4->points[i].x)
            {
                searchPoint = c_plane4->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[4].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane4->points[pointIdxNKNSearch[i]];
            pointlist[4].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane4->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane4->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane4->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane4->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane4->points.size() / 12);
        }*/
    }      
    path->points[4] = pointlist[index_].points[1];
    path->points[15] = pointlist[index_].points[pointlist[index_].points.size()-2];
    int j = pointlist[index_].points.size()/12;
    for (size_t i = 5; i < 15; i++)
    {
       path->points[i] = pointlist[index_].points[j];
       j =j+int(pointlist[index_].points.size()/12);
    }
    vector<pcl::PointCloud<pcl::PointXYZ>> no_mid;
    no_mid.resize(4);
    for (size_t i = 0; i < 5; i++)
    {
        if(i == index_)
        {
            continue;
        }
        if(i > index_)
        {
            no_mid[i-1] = pointlist[i];
        }
        if(i < index_)
        {
            no_mid[i] = pointlist[i];
        }
    }
    
    /****寻找最左侧2个点*****/
    max = 0;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(no_mid[index_].makeShared());
    int K = no_mid[index_].points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[20] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[21] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];


    /****  寻找U型左侧4个点 ****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
        
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[0] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[1] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4)]];
    path->points[2] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4) + int(pointIdxNKNSearch.size()/4)]];
    path->points[3] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];

    /*****寻找U型右侧4个点 *****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[16] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[17] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4)]];
    path->points[18] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4) + int(pointIdxNKNSearch.size()/4)]];
    path->points[19] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];
    
    /*****寻找右侧2个点 *****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[22] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[23] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];


    for (size_t i = 0; i < path->points.size(); i++)
    {
        cout << path->points[i].x << "," << path->points[i].y<<","<<path->points[i].z <<endl;
    }
    }
    if(arguments == 1)
    {
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/slishy/Code/PCD/hanjie/no_end_board.pcd",
                               *cloud); 
    pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>); 
	normEst.setInputCloud(cloud); 
	normEst.setRadiusSearch(0.005); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	boundEst.setInputCloud(cloud); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.setKSearch(100);
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
	std::cerr << "boundaries: " <<boundaries.points.size() << std::endl;
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < cloud->points.size(); i++) 
	{ 
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(cloud->points[i]); 
		} 
	}
     
    pcl::PointXYZ minpoint,maxpoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr U(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::getMinMax3D(*cloud_boundary,minpoint,maxpoint);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_boundary);         
    pass.setFilterFieldName("x");      
    pass.setFilterLimits(minpoint.x + 0.005,maxpoint.x - 0.005);    
    pass.filter(*U);
    pass.setInputCloud(U);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits((minpoint.z+maxpoint.z)/2,maxpoint.z);    
    pass.filter(*U); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr L(new pcl::PointCloud<pcl::PointXYZ>);  
    pass.setInputCloud(cloud_boundary);         
    pass.setFilterFieldName("x");      
    pass.setFilterLimits(minpoint.x + 0.005,maxpoint.x - 0.005);    
    pass.setNegative(true);
    pass.filter(*L);
    *cloud_boundary = *L +*U ;
    Pointviewer pv;
    pv.simpleVisN(cloud_boundary); 
    // find 24 points //
    *cloud = *cloud_boundary;
     PointProcess pp;
    pp.SetLeafSize(0.004);
    pp.DownSimple(cloud);

    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane);
    pcl::io::savePCDFile("c_plane.pcd",*c_plane);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane1(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane1);
    pcl::io::savePCDFile("c_plane1.pcd",*c_plane1);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane2);
    pcl::io::savePCDFile("c_plane2.pcd",*c_plane2);
     extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);


     seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane3(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane3);
    pcl::io::savePCDFile("c_plane3.pcd",*c_plane3);
     extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);



    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane4(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane4);
    pcl::io::savePCDFile("c_plane4.pcd",*c_plane4);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);
    //cout << cloud->points.size() <<endl;
    vector<pcl::PointCloud<pcl::PointXYZ>> pointlist;
    pointlist.resize(5);
    
    vector<float> x_distance(5);
    x_distance[0] = abs(c_plane->points[0].x - c_plane->points[c_plane->points.size()-1].x);
    x_distance[1] = abs(c_plane1->points[0].x - c_plane1->points[c_plane1->points.size()-1].x);
    x_distance[2] = abs(c_plane2->points[0].x - c_plane2->points[c_plane2->points.size()-1].x);
    x_distance[3] = abs(c_plane3->points[0].x - c_plane3->points[c_plane3->points.size()-1].x);
    x_distance[4] = abs(c_plane4->points[0].x - c_plane4->points[c_plane4->points.size()-1].x);
    float max = x_distance[0];
    int index_= 0;
    int index = 0;
/***** 寻找中间横着的点 *****/

    for (size_t i = 0; i < x_distance.size(); i++)
    {
        if (max < x_distance[i])
        {
            max = x_distance[i];
            index_ = i;
        }
    }
    
    pcl::PointXYZ minPoint, maxPoint;
    pcl::PointXYZ searchPoint;

    if(index == 0)
    {
        pcl::getMinMax3D(*c_plane, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane);
        int K = c_plane->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        for (size_t i = 0; i < c_plane->points.size(); i++)
        {
            if(maxPoint.x == c_plane->points[i].x)
            {
                searchPoint = c_plane->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[0].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            cloud->points[i] = c_plane->points[pointIdxNKNSearch[i]];
            pointlist[0].points[i] = cloud->points[i];
        }
        
       
       
        /*path->points[4] = c_plane->points[pointIdxNKNSearch[1]];
        path->points[15] = c_plane->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
        int j = c_plane->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane->points.size() / 12);
        }*/
    }   
    if(index == 0)
    {
        pcl::getMinMax3D(*c_plane1, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane1);
        int K = c_plane1->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane1->points.size(); i++)
        {
            if(maxPoint.x == c_plane1->points[i].x)
            {
                searchPoint = c_plane1->points[i];
            }
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
        pointlist[1].resize(pointIdxNKNSearch.size());
        cloud->points.resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane1->points[pointIdxNKNSearch[i]];
            pointlist[1].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane1->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane1->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane1->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane1->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane1->points.size() / 12);
        }*/
    } 
    if(index == 0)
    {
    
       pcl::getMinMax3D(*c_plane2, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane2);
        int K = c_plane2->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane2->points.size(); i++)
        {
            if(maxPoint.x == c_plane2->points[i].x)
            {
                searchPoint = c_plane2->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[2].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane2->points[pointIdxNKNSearch[i]];
            pointlist[2].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane2->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane2->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane2->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane2->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane2->points.size() / 12);
        }*/
    } 
    if(index == 0)
    {
    
       pcl::getMinMax3D(*c_plane3, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane3);
        int K = c_plane3->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane3->points.size(); i++)
        {
            if(maxPoint.x == c_plane3->points[i].x)
            {
                searchPoint = c_plane3->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
          cloud->points.resize(pointIdxNKNSearch.size());
          pointlist[3].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane3->points[pointIdxNKNSearch[i]];
            pointlist[3].points[i] = cloud->points[i];
        }

      
       /* path->points[4] = c_plane3->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane3->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane3->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane3->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane3->points.size() / 12);
        }*/
    }
    if(index == 0)
    {
    
        pcl::getMinMax3D(*c_plane4, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane4);
        int K = c_plane4->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane4->points.size(); i++)
        {
            if(maxPoint.x == c_plane4->points[i].x)
            {
                searchPoint = c_plane4->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[4].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane4->points[pointIdxNKNSearch[i]];
            pointlist[4].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane4->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane4->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane4->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane4->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane4->points.size() / 12);
        }*/
    }      
    path->points[4] = pointlist[index_].points[1];
    path->points[15] = pointlist[index_].points[pointlist[index_].points.size()-2];
    int j = pointlist[index_].points.size()/12;
    for (size_t i = 5; i < 15; i++)
    {
       path->points[i] = pointlist[index_].points[j];
       j =j+int(pointlist[index_].points.size()/12);
    }
    vector<pcl::PointCloud<pcl::PointXYZ>> no_mid;
    no_mid.resize(4);
    for (size_t i = 0; i < 5; i++)
    {
        if(i == index_)
        {
            continue;
        }
        if(i > index_)
        {
            no_mid[i-1] = pointlist[i];
        }
        if(i < index_)
        {
            no_mid[i] = pointlist[i];
        }
    }
    
    /****寻找最左侧2个点*****/
    max = 0;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(no_mid[index_].makeShared());
    int K = no_mid[index_].points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[20] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[21] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];


    /****  寻找U型左侧4个点 ****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            cout << max <<endl;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[0] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[1] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4)]];
    path->points[2] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4) + int(pointIdxNKNSearch.size()/4)]];
    path->points[3] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];

    /*****寻找U型右侧4个点 *****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            cout << max <<endl;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[16] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[17] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4)]];
    path->points[18] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4) + int(pointIdxNKNSearch.size()/4)]];
    path->points[19] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];
    
    /*****寻找右侧2个点 *****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            cout << max <<endl;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[22] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[23] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];


    for (size_t i = 0; i < path->points.size(); i++)
    {
        cout << path->points[i].x << "," << path->points[i].y<<","<<path->points[i].z <<endl;
    }

    }
    if(arguments == 2)
    {
        pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/slishy/Code/PCD/hanjie/with_end_board.pcd",
                               *cloud); 
    pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>); 
	normEst.setInputCloud(cloud); 
	normEst.setRadiusSearch(0.008); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	boundEst.setInputCloud(cloud); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.setKSearch(20);
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < cloud->points.size(); i++) 
	{ 
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(cloud->points[i]); 
		} 
	}
     
    pcl::PointXYZ minpoint,maxpoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr U(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::getMinMax3D(*cloud_boundary,minpoint,maxpoint);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_boundary);         
    pass.setFilterFieldName("x");      
    pass.setFilterLimits(minpoint.x + 0.01,maxpoint.x - 0.01);    
    pass.filter(*U);
    pass.setInputCloud(U);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(minpoint.z+0.007, maxpoint.z - 0.01);    
    pass.filter(*U); 
    *cloud_boundary = *U ;
    Pointviewer pv;
    pv.simpleVisN(cloud_boundary); 
    // find 24 points //
    *cloud = *cloud_boundary;
     PointProcess pp;
    pp.SetLeafSize(0.004);
    pp.DownSimple(cloud);

    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量

    pcl::ExtractIndices<pcl::PointXYZ> extract; 
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane);
    pcl::io::savePCDFile("c_plane.pcd",*c_plane);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane1(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane1);
    pcl::io::savePCDFile("c_plane1.pcd",*c_plane1);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);

    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane2);
    pcl::io::savePCDFile("c_plane2.pcd",*c_plane2);
     extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);


     seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane3(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane3);
    pcl::io::savePCDFile("c_plane3.pcd",*c_plane3);
     extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);



    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
    /*pp.SetStddevMulThresh(1);
    pp.SetK(50);
    pp.Removepoint(c_plane);*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane4(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);//false is extract inliers 
    extract.filter(*c_plane4);
    pcl::io::savePCDFile("c_plane4.pcd",*c_plane4);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);//false is extract inliers 
    extract.filter(*cloud);
    //cout << cloud->points.size() <<endl;
    vector<pcl::PointCloud<pcl::PointXYZ>> pointlist;
    pointlist.resize(5);
    
    vector<float> x_distance(5);
    x_distance[0] = abs(c_plane->points[0].x - c_plane->points[c_plane->points.size()-1].x);
    x_distance[1] = abs(c_plane1->points[0].x - c_plane1->points[c_plane1->points.size()-1].x);
    x_distance[2] = abs(c_plane2->points[0].x - c_plane2->points[c_plane2->points.size()-1].x);
    x_distance[3] = abs(c_plane3->points[0].x - c_plane3->points[c_plane3->points.size()-1].x);
    x_distance[4] = abs(c_plane4->points[0].x - c_plane4->points[c_plane4->points.size()-1].x);
    float max = x_distance[0];
    int index_= 0;
    int index = 0;
/***** 寻找中间横着的点 *****/

    for (size_t i = 0; i < x_distance.size(); i++)
    {
        if (max < x_distance[i])
        {
            max = x_distance[i];
            index_ = i;
        }
    }
    
    pcl::PointXYZ minPoint, maxPoint;
    pcl::PointXYZ searchPoint;

    if(index == 0)
    {
        pcl::getMinMax3D(*c_plane, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane);
        int K = c_plane->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        for (size_t i = 0; i < c_plane->points.size(); i++)
        {
            if(maxPoint.x == c_plane->points[i].x)
            {
                searchPoint = c_plane->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[0].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            cloud->points[i] = c_plane->points[pointIdxNKNSearch[i]];
            pointlist[0].points[i] = cloud->points[i];
        }
        
       
       
        /*path->points[4] = c_plane->points[pointIdxNKNSearch[1]];
        path->points[15] = c_plane->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
        int j = c_plane->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane->points.size() / 12);
        }*/
    }   
    if(index == 0)
    {
        pcl::getMinMax3D(*c_plane1, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane1);
        int K = c_plane1->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane1->points.size(); i++)
        {
            if(maxPoint.x == c_plane1->points[i].x)
            {
                searchPoint = c_plane1->points[i];
            }
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
        pointlist[1].resize(pointIdxNKNSearch.size());
        cloud->points.resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane1->points[pointIdxNKNSearch[i]];
            pointlist[1].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane1->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane1->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane1->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane1->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane1->points.size() / 12);
        }*/
    } 
    if(index == 0)
    {
    
       pcl::getMinMax3D(*c_plane2, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane2);
        int K = c_plane2->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane2->points.size(); i++)
        {
            if(maxPoint.x == c_plane2->points[i].x)
            {
                searchPoint = c_plane2->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[2].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane2->points[pointIdxNKNSearch[i]];
            pointlist[2].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane2->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane2->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane2->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane2->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane2->points.size() / 12);
        }*/
    } 
    if(index == 0)
    {
    
       pcl::getMinMax3D(*c_plane3, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane3);
        int K = c_plane3->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane3->points.size(); i++)
        {
            if(maxPoint.x == c_plane3->points[i].x)
            {
                searchPoint = c_plane3->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
          cloud->points.resize(pointIdxNKNSearch.size());
          pointlist[3].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane3->points[pointIdxNKNSearch[i]];
            pointlist[3].points[i] = cloud->points[i];
        }

      
       /* path->points[4] = c_plane3->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane3->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane3->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane3->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane3->points.size() / 12);
        }*/
    }
    if(index == 0)
    {
    
        pcl::getMinMax3D(*c_plane4, minPoint, maxPoint);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(c_plane4);
        int K = c_plane4->points.size();
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        
        for (size_t i = 0; i < c_plane4->points.size(); i++)
        {
            if(maxPoint.x == c_plane4->points[i].x)
            {
                searchPoint = c_plane4->points[i];
            } 
        }
        kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(pointIdxNKNSearch.size());
        pointlist[4].resize(pointIdxNKNSearch.size());
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
             cloud->points[i] = c_plane4->points[pointIdxNKNSearch[i]];
            pointlist[4].points[i] = cloud->points[i];
        }
        /*path->points[4] = c_plane4->points[pointIdxNKNSearch[1]];
         path->points[15] = c_plane4->points[pointIdxNKNSearch[pointIdxNKNSearch.size()-2]];
         int j = c_plane4->points.size()/12;
        for (size_t i = 5; i < 15; i++)
        {
            path->points[i] = c_plane4->points[pointIdxNKNSearch[j]];
            j =j+int(c_plane4->points.size() / 12);
        }*/
    }      
    path->points[4] = pointlist[index_].points[1];
    path->points[15] = pointlist[index_].points[pointlist[index_].points.size()-2];
    int j = pointlist[index_].points.size()/12;
    for (size_t i = 5; i < 15; i++)
    {
       path->points[i] = pointlist[index_].points[j];
       j =j+int(pointlist[index_].points.size()/12);
    }
    vector<pcl::PointCloud<pcl::PointXYZ>> no_mid;
    no_mid.resize(4);
    for (size_t i = 0; i < 5; i++)
    {
        if(i == index_)
        {
            continue;
        }
        if(i > index_)
        {
            no_mid[i-1] = pointlist[i];
        }
        if(i < index_)
        {
            no_mid[i] = pointlist[i];
        }
    }
    
    /****寻找最左侧2个点*****/
    max = 0;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(no_mid[index_].makeShared());
    int K = no_mid[index_].points.size();
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[20] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[21] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];


    /****  寻找U型左侧4个点 ****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[0] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[1] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4)]];
    path->points[2] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4) + int(pointIdxNKNSearch.size()/4)]];
    path->points[3] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];

    /*****寻找U型右侧4个点 *****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
         
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[16] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[17] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4)]];
    path->points[18] = no_mid[index_].points[pointIdxNKNSearch[int(pointIdxNKNSearch.size()/4) + int(pointIdxNKNSearch.size()/4)]];
    path->points[19] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];
    
    /*****寻找右侧2个点 *****/
    max = 0;
    no_mid[index_].points[0].x = -9999;
    for (size_t i = 0; i < 4; i++)
    {
        if(max < no_mid[i].points[0].x)
        {
            max = no_mid[i].points[0].x;
            
            index_ = i;
        }
    }
    pcl::getMinMax3D(*no_mid[index_].makeShared(), minPoint, maxPoint);
    kdtree.setInputCloud(no_mid[index_].makeShared());
    K = no_mid[index_].points.size();
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
    for (size_t i = 0; i < no_mid[index_].points.size(); i++)
    {
        if(maxPoint.z == no_mid[index_].points[i].z)
        {
            searchPoint = no_mid[index_].points[i];
        } 
    }
    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    path->points[22] = no_mid[index_].points[pointIdxNKNSearch[0]];
    path->points[23] = no_mid[index_].points[pointIdxNKNSearch[pointIdxNKNSearch.size() - 1]];


    for (size_t i = 0; i < path->points.size(); i++)
    {
        cout << path->points[i].x << "," << path->points[i].y<<","<<path->points[i].z <<endl;
    }

    }
}


