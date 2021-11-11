#include "PointProcess.h"
void PointProcess::DownSimple(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
{
    float Size = m_LeafSize;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(Size,Size,Size);
    sor.filter(*cloud_in);
}

 void PointProcess::DownSimple(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist)
 {
     float Size = m_LeafSize;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    for (size_t i = 0; i < cloudlist.size(); i++)
    {
        for (size_t j = 0; j < cloudlist[i].points.size(); j++)
        {
            cloud->points[j].x = cloudlist[i].points[j].x;
            cloud->points[j].y = cloudlist[i].points[j].y;
            cloud->points[j].z = cloudlist[i].points[j].z;
           
        }
        sor.setInputCloud(cloud);
        sor.setLeafSize(Size,Size,Size);
        sor.filter(*cloud);
        for (size_t k = 0; k < cloud->points.size(); k++)
        {
            cloudlist[i].points[k].x = cloud->points[k].x;
            cloudlist[i].points[k].y = cloud->points[k].y;
            cloudlist[i].points[k].z = cloud->points[k].z;
        }
    }
 }
 void PointProcess::LimitCenter2PointDistance(const vector<pcl::PointXYZ> center,vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist)
 {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	
    for (size_t i = 0; i < cloudlist.size(); i++)
    {
        kdtree.setInputCloud(cloudlist[i].makeShared());
	    int k = cloudlist[i].points.size();
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        inliers->indices.resize(k);
        /*if (cloudlist[i].points.size() == 0)
        {
            break;
        }*/
        kdtree.nearestKSearch(center[i],k,pointIdxNKNSearch,pointNKNSquaredDistance);
	    for (size_t j = 0; j < pointIdxNKNSearch.size(); j++)
	    {
		    if (sqrt(pointNKNSquaredDistance[j]) > m_Distance)
		    {
			    inliers->indices[j] = pointIdxNKNSearch[j];
		    }
	    }
        pcl::copyPointCloud(*cloudlist[i].makeShared(),*cloud);
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud);
       // pcl::io::savePCDFile("cloud12121.pcd",*cloud);
        //cloudlist[i].clear();
        cloudlist[i].width = cloud->width;
        cloudlist[i].height = cloud->height;
        cloudlist[i].is_dense = false;
        cloudlist[i].points.resize(cloudlist[i].width * cloudlist[i].height);
        for (size_t l = 0; l < cloud->points.size(); l++)
        {
            cloudlist[i].points[l].x = cloud->points[l].x;
            cloudlist[i].points[l].y = cloud->points[l].y;
            cloudlist[i].points[l].z = cloud->points[l].z;
        }
        
        //pcl::copyPointCloud(*cloud,*cloudlist[i].makeShared());
        //pcl::io::savePCDFile("cloud121211111.pcd",*cloudlist[i].makeShared());
        /*for (size_t n = 0; n < cloud->points.size(); n++)
        {
            cloudlist.resize(i+1);
            pcl::io::savePCDFile("cloud121211111.pcd",*cloudlist[i].makeShared());
            cloudlist[i].push_back(pcl::PointXYZ(cloud->points[n].x,cloud->points[n].y,cloud->points[n].z));
        }*/
    }
 }
 void PointProcess::Removepoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
 {
     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
     sor.setInputCloud(cloud_in);
     sor.setMeanK(m_K);
     sor.setStddevMulThresh(m_StddevMulThresh);
     sor.filter(*cloud_in);
 }
  void PointProcess::Removepoint(vector<pcl::PointCloud<pcl::PointXYZ>> &cloudlist)
 {
     for (size_t i = 0; i < cloudlist.size(); i++)
     {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::copyPointCloud(*cloudlist[i].makeShared(),*cloud);
     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
     sor.setInputCloud(cloud);
     sor.setMeanK(m_K);
     sor.setStddevMulThresh(m_StddevMulThresh);
     sor.filter(*cloud);
     //cloudlist[i].width = cloud->points.size();
     //cloudlist[i].points.resize(cloud->points.size());
     cloudlist[i].clear();
    //pcl::copyPointCloud(*cloud,*cloudlist[i].makeShared());
   
   for (size_t k = 0; k < cloud->points.size(); k++)
       {    

            cloudlist[i].push_back(cloud->points[k]);
            /*cloudlist[i].points[k].x = cloud->points[k].x;
            cloudlist[i].points[k].y = cloud->points[k].y;
            cloudlist[i].points[k].z = cloud->points[k].z;*/
        }
    }
 }
 
 void PointProcess::smoothxyz(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
 {  
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	// smooth
	mls.setComputeNormals(false);
	mls.setInputCloud(cloud_in);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.009);
	mls.process(*cloud_in_);
    pcl::copyPointCloud(*cloud_in_,*cloud_in);
 }
 void PointProcess::limitZ(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
 {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);         
    pass.setFilterFieldName("z");      
    pass.setFilterLimits(m_MinValues,m_MaxValues);    
    pass.filter(*cloud_in); 
 }
  void PointProcess::limitX(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
 {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);         
    pass.setFilterFieldName("x");      
    pass.setFilterLimits(m_MinValues,m_MaxValues);    
    pass.filter(*cloud_in); 
 }
  void PointProcess::limitY(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
 {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);         
    pass.setFilterFieldName("y");      
    pass.setFilterLimits(m_MinValues,m_MaxValues);    
    pass.filter(*cloud_in); 
 }
 
