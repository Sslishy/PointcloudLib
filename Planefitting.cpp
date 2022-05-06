#include "Planefitting.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

Planefitting::Planefitting()
{
	m_DistanceThreshold = 0.05;
	m_MaxIterations = 100;
	m_Ratio = 0.25;
}
void Planefitting::extractbynormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	std::vector<int> inliers;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_in));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(m_DistanceThreshold);
	ransac.computeModel();
	ransac.getInliers(inliers);
    pcl::copyPointCloud(*cloud_in, inliers, *cloud_out);
}

void Planefitting::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudlist) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_in,*cloud_copy);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);
	int num;
	num = this->m_num;
	seg.setNumberOfThreads(num);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(m_MaxIterations);
	seg.setDistanceThreshold(m_DistanceThreshold);
	seg.setProbability(0.99900); 
	
	int i = 0, nr_points = (int)cloud_copy->points.size();
	while (cloud_copy->points.size() > m_Ratio * nr_points)
	{
		 // Segment the largest planar component from the remaining cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(cloud_copy);
        seg.segment(*inliers_plane, *coefficients_plane);
        if (inliers_plane->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_copy);
        extract.setIndices(inliers_plane);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        if(cloud_p->points.size() < num)
		{
			return;
		}
        cloudlist.push_back(cloud_p);
        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_copy.swap(cloud_f);//更新
	}
}
void Planefitting::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,string Negative) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(m_MaxIterations);
	seg.setDistanceThreshold(m_DistanceThreshold);
	seg.setInputCloud(cloud_in);
	seg.segment(*inliers_plane, *coefficients_plane);
	extract.setInputCloud(cloud_in);
	extract.setIndices(inliers_plane);
	if (Negative == "false")
	{
		extract.setNegative(false);
		extract.filter(*cloud_out);
	}
	if (Negative == "true")
	{
		extract.setNegative(true);
		extract.filter(*cloud_out);
	}
	if (Negative == "segmentation")
	{
		extract.setNegative(false);
		extract.filter(*cloud_out);
		extract.setNegative(true);
		extract.filter(*cloud_in);
	}
}
