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
void Planefitting::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>>& cloudlist) {
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

	int i = 0, nr_points = (int)cloud_in->points.size();
	while (cloud_in->points.size() > m_Ratio * nr_points)
	{
		cloudlist.resize(i + 1);
		seg.setInputCloud(cloud_in);
		seg.segment(*inliers_plane, *coefficients_plane);
		
		if (inliers_plane->indices.size() == 0)
		{
			break;
		}
		extract.setInputCloud(cloud_in);
		extract.setIndices(inliers_plane);
		switch (i)
		{
		case 0:
			extract.setNegative(false);
			extract.filter(*cloud_mid);
			for (size_t i = 0; i < cloud_mid->points.size(); i++)
			{
				cloudlist[0].push_back(pcl::PointXYZ(cloud_mid->points[i].x, cloud_mid->points[i].y, cloud_mid->points[i].z));
			}
			//pcl::io::savePCDFile("cloud_mid.pcd", *cloud_mid);
			break;
		case 1:
			extract.setNegative(false);
			extract.filter(*cloud_mid1);
			for (size_t i = 0; i < cloud_mid1->points.size(); i++)
			{
				cloudlist[1].push_back(pcl::PointXYZ(cloud_mid1->points[i].x, cloud_mid1->points[i].y, cloud_mid1->points[i].z));
			}
			//pcl::io::savePCDFile("cloud_mid1.pcd", *cloud_mid1);
			break;
		default:
			break;
		}
		extract.setNegative(true);
		extract.filter(*cloud_out1);
		*cloud_in = *cloud_out1;
		i++;
		if (i == 2)
		{
			break;
		}
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
	
	
	
	
	
}
