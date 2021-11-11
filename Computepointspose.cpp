#include "Computepointspose.h"

void Computepointspose::computePickPoints(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, vector<pcl::PointXYZ>& center)
{
	Eigen::Vector4f center_;
	center.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
		pcl::compute3DCentroid(cloudlist[i], center_);
		center[i] = pcl::PointXYZ(center_[0], center_[1], center_[2]);
	}
}
Computepointspose::Computepointspose()
{
	m_K = 99999;
}
void Computepointspose::computePointNormal(const vector<pcl::PointCloud<pcl::PointXYZ>> cloudlist, const vector<pcl::PointXYZ> center,  vector<pcl::Normal>& N)
{
	vector<vector<int>> pointIdxNKNSearch1;
	vector<int> pointIdxNKNSearch;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<float> pointNKNSquaredDistance;
	pointIdxNKNSearch1.reserve(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
		kdtree.setInputCloud(cloudlist[i].makeShared());
		//kdtree.radiusSearch(center[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance);
		kdtree.nearestKSearch(center[i], m_K, pointIdxNKNSearch, pointNKNSquaredDistance);
		pointIdxNKNSearch1.push_back(pointIdxNKNSearch);
	}
	Eigen::Matrix3f covariance1;
	Eigen::Vector4f center1;
	Eigen::Vector4f plane_parameters1;
	vector<Eigen::Vector4f> plane_parameters;
	vector<float> curvature;
	float curvature1;
	curvature.reserve(cloudlist.size());
	plane_parameters.reserve(cloudlist.size());
	N.reserve(cloudlist.size());
	m_N.resize(cloudlist.size());
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
		computeMeanAndCovarianceMatrix(cloudlist[i], pointIdxNKNSearch1[i], covariance1, center1);
		pcl::solvePlaneParameters(covariance1, center1, plane_parameters1, curvature1);
		plane_parameters.push_back(plane_parameters1);
		curvature.push_back(curvature1);
		N.push_back(pcl::Normal(plane_parameters[i][0], plane_parameters[i][1], plane_parameters[i][2]));
		flipNormalTowardsViewpoint(center[i], 0.0, 0.0, 9999.0, N[i].normal_x, N[i].normal_y, N[i].normal_z);
		m_N[i] = N[i];
	}
	
}
void Computepointspose::Setfilternormalangle(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist,const float angle)
{
	vector<float> x;
	x.resize(cloudlist.size());
	vector<int>::iterator iter;
	float angle_ = (CV_PI/180)*angle;
	for (size_t i = 0; i < cloudlist.size(); i++)
	{
		x[i] = acos(abs(m_N[i].normal_z)); 
		if (x[i] > angle_)
		{
			cloudlist[i].clear();
		}
		
	}
}
void Computepointspose::computePickPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ& center_out)
{
	Eigen::Vector4f center_;
	pcl::compute3DCentroid(*cloud_in, center_);
	center_out = pcl::PointXYZ(center_[0], center_[1], center_[2]);
	
}
void Computepointspose::computePointNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointXYZ center, pcl::Normal &N)
{
	vector<int> pointIdxNKNSearch;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<float> pointNKNSquaredDistance;
	kdtree.setInputCloud(cloud_in);
	//kdtree.radiusSearch(center[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance);
	kdtree.nearestKSearch(center, m_K, pointIdxNKNSearch, pointNKNSquaredDistance);
	Eigen::Matrix3f covariance1;
	Eigen::Vector4f center1;
	Eigen::Vector4f plane_parameters1;
	float curvature1;
	computeMeanAndCovarianceMatrix(*cloud_in, pointIdxNKNSearch, covariance1, center1);
	pcl::solvePlaneParameters(covariance1, center1, plane_parameters1, curvature1);
	N.normal_x = plane_parameters1[0];
	N.normal_y = plane_parameters1[1];
	N.normal_z = plane_parameters1[2];
	flipNormalTowardsViewpoint(center, 0.0, 0.0, 9999.0, N.normal_x, N.normal_y, N.normal_z);
}
void Computepointspose::Setlimitangle(const float angle,vector<pcl::Normal> &Normal_Z)
{
	vector<float> outangle;
	float angleradian = (CV_PI/180)*angle;
	float PointToNormalzDistance;
	pcl::Normal changenormalz;
	Getangle(outangle);
	float lenZ;
	float newNormalzlen;
	for (size_t i = 0; i < outangle.size(); i++)
	{
		if (outangle[i] > angle)
		{
			lenZ = cos(outangle[i]);
			PointToNormalzDistance = tan(angleradian) * lenZ;
			//normalz to OXY projection distance 
			float len1 = sqrt(pow(m_N[i].normal_x,2) + pow(m_N[i].normal_y,2));
			changenormalz.normal_x = m_N[i].normal_x * PointToNormalzDistance / len1;
			changenormalz.normal_y = m_N[i].normal_y * PointToNormalzDistance / len1;
			changenormalz.normal_z = m_N[i].normal_z;
			newNormalzlen = sqrt(pow(changenormalz.normal_x,2) + pow(changenormalz.normal_y,2) + pow(changenormalz.normal_z,2));
			Normal_Z[i].normal_x = changenormalz.normal_x / newNormalzlen;
			Normal_Z[i].normal_y = changenormalz.normal_y / newNormalzlen;
			Normal_Z[i].normal_z = changenormalz.normal_z / newNormalzlen;
		}	
	}
}

void Computepointspose::SetfixedAngle(const float angle , vector<pcl::Normal> &Normal_Z)
{
	
	float angleradian = (CV_PI/180)*angle;
	float PointToNormalzDistance;
	pcl::Normal changenormalz;
	float lenZ;
	float newNormalzlen;
	for (size_t i = 0; i < Normal_Z.size(); i++)
	{
		lenZ = cos(angleradian);
		PointToNormalzDistance = tan(angleradian) * lenZ;
		//normalz to OXY projection distance 
		float len1 = sqrt(pow(Normal_Z[i].normal_x,2) + pow(Normal_Z[i].normal_y,2));
		changenormalz.normal_x = Normal_Z[i].normal_x * PointToNormalzDistance / len1;
		changenormalz.normal_y = Normal_Z[i].normal_y * PointToNormalzDistance / len1;
		changenormalz.normal_z = Normal_Z[i].normal_z;
		newNormalzlen = sqrt(pow(changenormalz.normal_x,2) + pow(changenormalz.normal_y,2) + pow(changenormalz.normal_z,2));
		Normal_Z[i].normal_x = changenormalz.normal_x / newNormalzlen;
		Normal_Z[i].normal_y = changenormalz.normal_y / newNormalzlen;
		Normal_Z[i].normal_z = changenormalz.normal_z / newNormalzlen;
		
	}
}
void Computepointspose::SetfixedAngle(const float angle , pcl::Normal &Normal_Z)
{
	
	float angleradian = (CV_PI/180)*angle;
	float PointToNormalzDistance;
	pcl::Normal changenormalz;
	float lenZ;
	float newNormalzlen;
	lenZ = cos(angleradian);
	PointToNormalzDistance = tan(angleradian) * lenZ;
	//normalz to OXY projection distance 
	float len1 = sqrt(pow(Normal_Z.normal_x,2) + pow(Normal_Z.normal_y,2));
	changenormalz.normal_x = Normal_Z.normal_x * PointToNormalzDistance / len1;
	changenormalz.normal_y = Normal_Z.normal_y * PointToNormalzDistance / len1;
	changenormalz.normal_z = Normal_Z.normal_z;
	newNormalzlen = sqrt(pow(changenormalz.normal_x,2) + pow(changenormalz.normal_y,2) + pow(changenormalz.normal_z,2));
	Normal_Z.normal_x = changenormalz.normal_x / newNormalzlen;
	Normal_Z.normal_y = changenormalz.normal_y / newNormalzlen;
	Normal_Z.normal_z = changenormalz.normal_z / newNormalzlen;
	
}
Computepointspose::~Computepointspose(){
}