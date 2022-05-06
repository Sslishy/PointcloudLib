#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv) {


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string strarguments = argv[1];
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>(strarguments,*cloud);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x *= 0.001;
        cloud->points[i].y *= 0.001;
        cloud->points[i].z *= 0.001;
    }
    pcl::io::savePCDFile("cloud.pcd",*cloud);
}

