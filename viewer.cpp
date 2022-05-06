#include <pcl/visualization/pcl_visualizer.h> 
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
 
int user_data;
 
void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;
 
}
 
void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);
 
	//FIXME: possible race condition here:
	user_data++;
}
 
int
main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
 
	// Fill in the cloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
 
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
 
	std::cerr << "Cloud before projection: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
 
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 1;
	coefficients->values[2] = 0;
	coefficients->values[3] = 10;
 
	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
 
	std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << std::endl;
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();
	
 
	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(1.0, 0.5, 1.0, v1);
	viewer.addText("Cloud before voxelgrid filtering", 10, 10, "v1 test", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud,0,255,0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud", v1);
	viewer.addCoordinateSystem(1,cloud->points[0].x, cloud->points[0].y, cloud->points[0].z,"v1",v1);
 
	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(1.0, 0.5, 1.0, v2);
	viewer.addText("Cloud after projection", 10, 10, "v2 test", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_projection_color(cloud_projected,0,0, 255);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, cloud_projection_color, "cloud_projected", v2);
	viewer.addCoordinateSystem(1, cloud_projected->points[0].x, cloud_projected->points[0].y, cloud_projected->points[0].z, "v1",v2);
 
 
 
 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_projected");
 
 
 
	while (!viewer.wasStopped())
	{
		//在此处可以添加其他处理
		
		viewer.spinOnce(100);
		
	}
 
 
	return 0;
}