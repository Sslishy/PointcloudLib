#include "Pointviewer.h"
pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}
pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1)
{
	
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud,0.0,0.0,255.0),"cloud");
	viewer->addPointCloud(cloud1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud1,0.0,255.0,0.0),"cloud1");
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}
pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,pcl::PointXYZ center)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud,0.0,0.0,255.0),"cloud");
	viewer->addPointCloud(cloud1,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud1,0.0,255.0,0.0),"cloud1");
	viewer->addSphere(center,0.0025,"sphere");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}

__attribute__((force_align_arg_pointer))pcl::visualization::PCLVisualizer::Ptr Pointviewer::simpleVisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr center,vector<cv::Mat> RPY)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//Mat R = eulerAnglesToRotationMatrix(RPY);
	for (size_t i = 0; i < center->points.size(); i++)
	{
		Mat R = RPY[i];
	Eigen::Matrix4f m4f_transform1;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 4) <<
			-R.at<double>(0, 0), -R.at<double>(0, 1), -R.at<double>(0, 2), center->points[i].x,
			-R.at<double>(1, 0), -R.at<double>(1, 1), -R.at<double>(1, 2), center->points[i].y,
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), center->points[i].z,
			0, 0, 0, 1
			);
	cv::cv2eigen(R1, m4f_transform1);
	Eigen::Transform<float, 3, Eigen::Affine> a3f_transform(m4f_transform1);
	viewer->addCoordinateSystem(0.04, a3f_transform, to_string(i));
	viewer->addText3D("p"+to_string(i),center->points[i],0.002);
	}
	viewer->addCoordinateSystem(0.5);
	viewer->addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud,255.0,140.0,0.0),"cloud");
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
	return(viewer);
}