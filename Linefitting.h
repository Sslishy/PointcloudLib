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
#include<pcl/features/boundary.h>
using namespace std;
class Linefitting
{
private:
    int m_num;
    int m_MaxIterations;
    float m_DistanceThreshold;
    float m_Ratio;
    float m_boundaryKSearch;
    float m_boundaryRadiusSearch;
public:
    Linefitting(/* args */);
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
    const void SetNumofThreshold( const int num)
	{
		this->m_num = num;		
	}
    const void SetboundaryKSearch( const int boundaryKSearch)
	{
		this->m_boundaryKSearch = boundaryKSearch;		
	}
    const void SetboundaryRadiusSearch( const float boundaryRadiusSearch)
	{
		this->m_boundaryRadiusSearch = boundaryRadiusSearch;		
	}
    void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudlist);
    ~Linefitting();
};

Linefitting::Linefitting(/* args */)
{
}

Linefitting::~Linefitting()
{
}
