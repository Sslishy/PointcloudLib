#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
using namespace std;
class PointProcess
{
private:
    float m_Distance;
    float m_LeafSize;
    float m_K;
    float m_StddevMulThresh;
    float m_MinValues;
    float m_MaxValues;
public:
    PointProcess(/* args */);
    ~PointProcess();
    const inline void SetK(float K)
    {
        m_K = K;
    }
     const inline void MinValues(float Min)
    {
        m_MinValues = Min;
    }
     const inline void MaxValues(float Max)
    {
        m_MaxValues = Max;
    }
    const inline void SetStddevMulThresh(float StddevMulThresh)
    {
        m_StddevMulThresh = StddevMulThresh;
    }
    const inline void SetLeafSize(float LeafSize)
    {
        m_LeafSize = LeafSize;
    }
    const inline void SetDistance(float Distance)
    {
        m_Distance = Distance * 0.001;
    }

    void DownSimple(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);
    void DownSimple(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist);
    void Removepoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    void Removepoint(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist);
    void LimitCenter2PointDistance(const vector<pcl::PointXYZ> center,vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist);
    void smoothxyz(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    void limitZ(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);
    void limitX(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);
    void limitY(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);
};

PointProcess::PointProcess(/* args */)
{
    m_LeafSize = 0.01;
    m_K = 50;
    m_StddevMulThresh = 0.05;
   
    
}

PointProcess::~PointProcess()
{
}
