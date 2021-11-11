#pragma once
#include<iostream>
#include<pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include<pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
using namespace std;
class Planefitting
{
public:
	const  int GetMaxIterations() const {
		return m_MaxIterations;
	}
	const  float GetDistanceThreshold() const {
		return m_DistanceThreshold;
	}
	 void SetMaxIterations(const int& MaxIterations) {
		this->m_MaxIterations = MaxIterations;
	}
	 void SetDistanceThreshold(const float& DistanceThreshold) {
		this->m_DistanceThreshold = DistanceThreshold;
	}
	const void SetRatio( const float Ratio)
	{
		this->m_Ratio = Ratio;		
	}
	Planefitting();
	void extractbynormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out); 
	void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>>& cloudlist);
	void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,string Negative);
	
	
private:
	int	m_MaxIterations;
	float m_DistanceThreshold;
	float m_Ratio;

};

