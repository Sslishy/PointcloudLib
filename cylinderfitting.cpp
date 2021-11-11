#include "cylinderfitting.h"

cylinderfitting::cylinderfitting()
{
		m_NormalDistanceWeight = 0.1;
		m_MaxIterations = 200;
 		m_DistanceThreshold = 0.05;
	    m_MinRadiusLimits = 0;
	    m_MaxRadiusLimits = 0.1;
}
void cylinderfitting::extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>>& cloudlist)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_in);
	ne.setKSearch(50);
	ne.compute(*cloud_normal);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(m_NormalDistanceWeight);
	seg.setMaxIterations(m_MaxIterations);
	seg.setDistanceThreshold(m_DistanceThreshold);
	seg.setRadiusLimits(m_MinRadiusLimits, m_MaxRadiusLimits);
	seg.setInputCloud(cloud_in);
	seg.setInputNormals(cloud_normal);
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	extract.setInputCloud(cloud_in);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	extract.filter(*cloud_mid);
	cloudlist.resize(1);
	for (size_t i = 0; i < cloud_mid->points.size(); i++)
	{
		cloudlist[0].push_back(pcl::PointXYZ(cloud_mid->points[i].x, cloud_mid->points[i].y, cloud_mid->points[i].z));
	}
}