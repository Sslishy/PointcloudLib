#include <iostream>
#include <thread>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include"Planefitting.h"
#include"Computepointspose.h"
#include"PointProcess.h"
#include"computeangle.h"
#include"PointCloudAligment.h"
#include"Pointviewer.h"
using namespace std::chrono_literals;
void GetCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,pcl::PointXYZ &mid23,pcl::PointXYZ &mid45)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f pcacentroid;
	pcl::compute3DCentroid(*cloud_in, pcacentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_in, pcacentroid, covariance);
	//计算矩阵的特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcacentroid.head<3>());
	pcl::transformPointCloud(*cloud_in, *cloud1, projectionTransform);

	vector<cv::Point2f> point22d;
	point22d.resize(cloud1->points.size());
	//pcl::io::savePCDFileASCII("cloud",*cloud1);
	for (size_t i = 0; i < cloud1->points.size(); i++)
	{
		point22d[i].x = cloud1->points[i].z+100;
		point22d[i].y = cloud1->points[i].y+100;
	}
	cv::RotatedRect pointbox = minAreaRect(point22d);
	Point2f rect[4];
	pointbox.points(rect);
	pcl::PointXYZ point2,point3,point4,point5,pointcenter_;
	pcl::visualization::PCLVisualizer viewer("viewer");
	//getCross(rect[0].x,rect[0].y,rect[2].x,rect[2].y,rect[1].x,rect[1].y,rect[3].x,rect[3].y,centerxy);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudmid(new pcl::PointCloud<pcl::PointXYZ>);
	cloudmid->points.resize(1);
	Point2f c = pointbox.center;
	//viewer.addPointCloud(cloud1,"cloud");
	//viewer.addCoordinateSystem();	
	pointcenter_.x = 0;
	pointcenter_.y = c.y - 100;
	pointcenter_.z = c.x - 100;
	cloudmid->points[0].x = pointcenter_.x;
	cloudmid->points[0].y = pointcenter_.y;
	cloudmid->points[0].z = pointcenter_.z;
	viewer.addPointCloud(cloud1,"cloud");
	viewer.addCoordinateSystem();	
	point2.x = 0;
	point2.y = rect[0].y - 100;
	point2.z = rect[0].x - 100;
	point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;
    
    pcl::PointXYZ pointo;
    float r,r1,len,radius;
    r = sqrt((point2.x -point5.x)*(point2.x -point5.x) + (point2.y -point5.y)*(point2.y -point5.y) + (point2.z -point5.z)*(point2.z -point5.z));
    r1 = sqrt((point2.x -point3.x)*(point2.x -point3.x) + (point2.y -point3.y)*(point2.y -point3.y) + (point2.z -point3.z)*(point2.z -point3.z));
    if (r > r1)
    {
    mid23.x = (point2.x +point3.x)/2; 
    mid23.y = (point2.y +point3.y)/2; 
    mid23.z = (point2.z +point3.z)/2; 
    mid45.x = (point4.x + point5.x)/2;
    mid45.y = (point4.y + point5.y)/2;
    mid45.z = (point4.z + point5.z)/2;
    viewer.addSphere(mid23,r1/2,"5");
    viewer.addSphere(mid45,r1/2,"6");
    radius = r1;
    }
    else
    { 
    mid23.x = (point2.x +point5.x)/2; 
    mid23.y = (point2.y +point5.y)/2; 
    mid23.z = (point2.z +point5.z)/2; 
    mid45.x = (point3.x + point4.x)/2;
    mid45.y = (point3.y + point4.y)/2;
    mid45.z = (point3.z + point4.z)/2;
     viewer.addSphere(mid23,r/2,"5");
    viewer.addSphere(mid45,r/2,"6");
    radius =r;
    }
    
     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     kdtree.setInputCloud(cloud1);
     std::vector<int> pointIdxRadiusSearch;
     pointIdxRadiusSearch.resize(9999);
    std::vector<float> pointRadiusSquaredDistance;
    pointRadiusSquaredDistance.resize(9999);
    len = sqrt((mid23.x -mid45.x)*(mid23.x -mid45.x) + (mid23.y -mid45.y)*(mid23.y -mid45.y) + (mid23.z -mid45.z)*(mid23.z -mid45.z));
    float j = 0.005;
    int maxpointcloud = 0;
	pcl::PointXYZ pointcenter;
    for (size_t i = 0; i < 999;i++)
    {
       pointo.x = j/len*(mid23.x - mid45.x)+mid45.x;
       pointo.y = j/len*(mid23.y - mid45.y)+mid45.y;
       pointo.z = j/len*(mid23.z - mid45.z)+mid45.z;
       j = j + 0.005;
       kdtree.radiusSearch(pointo, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      // maxpointcloud = pointIdxRadiusSearch.size();
       if (maxpointcloud < pointIdxRadiusSearch.size())
       {
          maxpointcloud = pointIdxRadiusSearch.size();
		  pointcenter = pointo;
       }
       if (j > len)
       {
           break;
       }
       viewer.addSphere(pointo,0.005,to_string(i)+"xyo");
    }
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcentercloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	kdtree.setInputCloud(cloud1);
    kdtree.radiusSearch(pointcenter, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	pointcentercloud->points.resize(pointIdxRadiusSearch.size());
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
	{
		pointcentercloud->points[i].x = cloud1->points[pointIdxRadiusSearch[i]].x;
		pointcentercloud->points[i].y = cloud1->points[pointIdxRadiusSearch[i]].y;
		pointcentercloud->points[i].z = cloud1->points[pointIdxRadiusSearch[i]].z;
	}
	Computepointspose cp;
	pcl::transformPointCloud(*pointcentercloud, *pointcentercloud, projectionTransform.inverse());

	cp.computePickPoints(pointcentercloud,pointcenter);
	pcl::Normal normal;
	cp.computePointNormal(pointcentercloud,pointcenter,normal);
	*cloud_out = *pointcentercloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud23(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud45(new pcl::PointCloud<pcl::PointXYZ>);
	cloud23->points.resize(1);
	cloud45->points.resize(1);
	cloud23->points[0] = mid23;
	cloud45->points[0] = mid45;
	pcl::transformPointCloud(*cloud23, *cloud23, projectionTransform.inverse());
	pcl::transformPointCloud(*cloud45, *cloud45, projectionTransform.inverse());
	mid23 = cloud23->points[0];
	mid45 = cloud45->points[0];
	/*point3.x = 0;
	point3.y = rect[1].y - 100;
	point3.z = rect[1].x - 100;
	point4.x = 0;
	point4.y = rect[2].y - 100;
	point4.z = rect[2].x - 100;
	point5.x = 0;
	point5.y = rect[3].y - 100;
	point5.z = rect[3].x - 100;*/
	viewer.addSphere(point2,0.005,"1");
	viewer.addSphere(point3,0.005,"2");
	viewer.addSphere(point4,0.005,"3");
	viewer.addSphere(point5,0.005,"4");
	viewer.addLine(point2,point3,"line");
	viewer.addLine(point3,point4,"line1");
	viewer.addLine(point4,point5,"line2");
	viewer.addLine(point5,point2,"line3");
	//viewer.addLine(point2,point4,"l1");
	//viewer.addLine(point3,point5,"l2");
	//viewer.addSphere(pointcenter_,0.02,"5");
}
void Getangle(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,Mat &rpy)
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_in);
    feature_extractor.compute();
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
	feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    // 获取OBB盒子
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    // 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	
	rpy = (cv::Mat_<double>(3, 3) <<
	major_vector(0), major_vector(1), major_vector(2), 
	middle_vector(0), middle_vector(1), middle_vector(2), 
	minor_vector(0), minor_vector(1), minor_vector(2)
	);;
}
int
main(int argc, char **argv) {
	string f = "false";
	string path1= argv[1];
	string path = "/home/slishy/Code/PCD/qianzi/" + path1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ center;
	pcl::Normal normal;
	vector<pcl::Normal> normal_Z(1),normal_X(1),normal_Y(1);
	vector<cv::Vec3f> RPYList(1);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *target_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    for (size_t i = 0; i < target_cloud->points.size(); i++)
    {
        target_cloud->points[i].x = target_cloud->points[i].x *0.001;
        target_cloud->points[i].y = target_cloud->points[i].y *0.001;
        target_cloud->points[i].z = target_cloud->points[i].z *0.001;
    }
    
	
	//提取平面点云
	Planefitting plane;
	plane.SetDistanceThreshold(0.002);
	plane.SetMaxIterations(100);
	plane.extract(target_cloud,target_cloud_,f);
    PointProcess pp;
    pp.SetStddevMulThresh(1);
	pp.SetK(300);
    pp.Removepoint(target_cloud_);
    Computepointspose cp;
	pcl::PointXYZ mid23,mid45,xpoint;
    GetCenter(target_cloud_,target_cloud_out,mid23,mid45);
	cp.computePickPoints(target_cloud_out,center);
	cp.computePointNormal(target_cloud_out,center,normal);
	computeangle ca;
	normal_Z[0] = normal;
	ca.ComputeDirection(normal_Z,normal_X,normal_Y);
	ca.computeRPY(normal_Z,normal_X,normal_Y,RPYList);
	float d1,d2;
	d1 = sqrt((center.x -mid23.x)*(center.x -mid23.x) + (center.y -mid23.y)*(center.y -mid23.y) + (center.z -mid23.z)*(center.z -mid23.z));
	d2 = sqrt((center.x -mid45.x)*(center.x -mid45.x) + (center.y -mid45.y)*(center.y -mid45.y) + (center.z -mid45.z)*(center.z -mid45.z));
	Mat rpyreal;
	if (d1 > d2)
	{
		xpoint.x = (mid23.x - center.x)/d1 + center.x;
		xpoint.y = (mid23.y - center.y)/d1 + center.y;
		xpoint.z = (mid23.z - center.z)/d1 + center.z;
		cout << sqrt((center.x -xpoint.x)*(center.x -xpoint.x) + (center.y -xpoint.y)*(center.y -xpoint.y) + (center.z -xpoint.z)*(center.z -xpoint.z)) <<endl;
		Eigen::Vector3d rx(xpoint.x - center.x, xpoint.y - center.y, xpoint.z - center.z),rz(normal.normal_x,normal.normal_y,normal.normal_z);
		Eigen::Vector3d ry = rz.cross(rx);
			Mat rpy = (cv::Mat_<double>(3, 3) <<
		rx[0], rx[1], rx[2], 
		ry[0], ry[1], ry[2], 
		rz[0], rz[1], rz[2]
		);;
		cout << ca.rotationMatrixToEulerAngles(rpy)<<endl;
		/*Vec3b rz = ca.rotationMatrixToEulerAngles(rpy);
		cout << rz <<endl;
		rpyreal = ca.eulerAnglesToRotationMatrix(RPYList[0]);
		RPYList[0] = Vec3b(0,0,rz[0]);
		cout << RPYList[0] <<endl;
		rpy = ca.eulerAnglesToRotationMatrix(RPYList[0]);
		rpyreal = rpyreal*rpy ;
		cout << "d1 > d2" <<endl;*/
		rpyreal = ca.eulerAnglesToRotationMatrix(ca.rotationMatrixToEulerAngles(rpy));
		  Pointviewer pv;
    pv.simpleVisN(target_cloud_out,target_cloud_,center,rx,ry,rz);
	}
	else
	{
		xpoint.x = (mid45.x - center.x)/d2 + center.x;
		xpoint.y = (mid45.y - center.y)/d2 + center.y;
		xpoint.z = (mid45.z - center.z)/d2 + center.z;
		cout << sqrt((center.x -xpoint.x)*(center.x -xpoint.x) + (center.y -xpoint.y)*(center.y -xpoint.y) + (center.z -xpoint.z)*(center.z -xpoint.z)) <<endl;
			/*Mat rpy = (cv::Mat_<double>(3, 3) <<
		1, 0, 0, 
		0, 1, 0, 
		center.x -xpoint.x, center.y -xpoint.y, center.z -xpoint.z
		);;
		Vec3b rz = ca.rotationMatrixToEulerAngles(rpy);
		cout << rz <<endl;
		rpyreal = ca.eulerAnglesToRotationMatrix(RPYList[0]);
		RPYList[0] = Vec3b(0,0,rz[0]);
		cout << RPYList[0] <<endl;
		rpy = ca.eulerAnglesToRotationMatrix(RPYList[0]);
		rpyreal = rpyreal*rpy;
		cout << "d2 > d1" <<endl;*/
	Eigen::Vector3d rx(xpoint.x - center.x, xpoint.y - center.y, xpoint.z - center.z),rz(normal.normal_x,normal.normal_y,normal.normal_z);
		Eigen::Vector3d ry = rz.cross(rx);
			Mat rpy = (cv::Mat_<double>(3, 3) <<
		rx[0], rx[1], rx[2], 
		ry[0], ry[1], ry[2], 
		rz[0], rz[1], rz[2]
		);;
		cout << ca.rotationMatrixToEulerAngles(rpy)<<endl;
		rpyreal = ca.eulerAnglesToRotationMatrix(ca.rotationMatrixToEulerAngles(rpy));
		  Pointviewer pv;
    pv.simpleVisN(target_cloud_out,target_cloud_,center,rx,ry,rz);
	}
	

	cout << center <<endl;
  
}
