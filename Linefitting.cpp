#include"Linefitting.h"
void Linefitting::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudlist) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_in,*cloud_copy);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
    normEst.setInputCloud(cloud_copy); 
    float RS = this->m_boundaryRadiusSearch;
   
    normEst.setRadiusSearch(RS); //设置法线估计的半径
    normEst.compute(*normals); //将法线估计结果保存至normals
    boundEst.setInputCloud(cloud_copy); //设置输入的点云
    boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
    int KS = this->m_boundaryKSearch;
   
    boundEst.setKSearch(KS);
    
    boundEst.compute(boundaries); //将边界估计结果保存在boundaries
    //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
    for(int i = 0; i < cloud_copy->points.size(); i++) 
    { 
        if(boundaries[i].boundary_point > 0) 
        { 
            cloud_boundary->push_back(cloud_copy->points[i]); 
        } 
    }
    pcl::io::savePCDFile("cloudboundary.pcd",*cloud_boundary);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);
	int num;
	num = this->m_num;
	seg.setNumberOfThreads(num);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(m_MaxIterations);
	seg.setDistanceThreshold(m_DistanceThreshold);
	seg.setProbability(0.99900); 
	int i = 0, nr_points = (int)cloud_boundary->points.size();
	while (cloud_boundary->points.size() > m_Ratio * nr_points)
	{
		 // Segment the largest planar component from the remaining cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(cloud_boundary);
        seg.segment(*inliers_plane, *coefficients_plane);
        if (inliers_plane->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_boundary);
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
        cloud_boundary.swap(cloud_f);//更新
	}
}