#pragma once
#include <pcl/io/pcd_io.h>
#include<pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
using namespace std;
class cylinderfitting
{
public:
	cylinderfitting();
	const inline void SetNormalDistanceWeight(float NormalDistanceWeight)
	{
		m_NormalDistanceWeight = NormalDistanceWeight;
	}
	const inline void SetMaxIterations(int MaxIterations)
	{
		m_MaxIterations = MaxIterations;
	}
	const inline void SetDistanceThreshold(float DistanceThreshold)
	{
		m_DistanceThreshold = DistanceThreshold;
	}
	const inline void SetMinRadiusLimits(float MinRadiusLimits)
	{
		m_MinRadiusLimits = MinRadiusLimits;
	}
	const inline void SetMaxRadiusLimits(float MaxRadiusLimits)
	{
		m_MaxRadiusLimits = MaxRadiusLimits;
	}
	void extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>>& cloudlist);
private:
	float m_NormalDistanceWeight;
	int m_MaxIterations;
	float m_DistanceThreshold;
	float m_MinRadiusLimits;
	float m_MaxRadiusLimits;
};

