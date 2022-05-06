#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/search/kdtree.h>
#include"PointProcess.h"
#include <pcl/filters/random_sample.h> // 采取固定数量的点云
#include <pcl/registration/ia_ransac.h>// 采样一致性
#include <pcl/registration/icp.h>      // icp配准
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h> // 可视化
#include <time.h> // 时间


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 点云可视化
void visualize_pcd(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{
	//创建初始化目标
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h(pcd_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h(pcd_tgt, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> final_h(pcd_final, 0, 0, 255);
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
	viewer.addPointCloud(pcd_final, final_h, "final cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		
	}
}


// 计算特征点的3DSC描述子
void computeKeyPoints3DSC(PointCloud::Ptr& cloud_in, PointCloud::Ptr& key_cloud, pcl::PointCloud<pcl::ShapeContext1980>::Ptr & dsc)
{
	//------------------计算法线----------------------
	pcl::NormalEstimationOMP<PointT, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(6);//设置openMP的线程数
	n.setInputCloud(key_cloud);
	n.setSearchSurface(cloud_in);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);
	cout << "法线计算完毕！！！" << endl;
	//-------------------计算3dsc-----------------------
	pcl::ShapeContext3DEstimation<PointT, pcl::Normal, pcl::ShapeContext1980> sc;
	sc.setInputCloud(key_cloud);
	sc.setInputNormals(normals);
	//kdTree加速
	sc.setSearchMethod(tree);
	sc.setMinimalRadius(0.02);     // 搜索球面(Rmin)的最小半径值。
	sc.setRadiusSearch(0.03);      // 设置用于确定用于特征估计的最近邻居的球体半径。
	sc.setPointDensityRadius(0.02);// 这个半径用于计算局部点密度=这个半径内的点数。
	sc.compute(*dsc);
	cout << "3DSC特征描述子计算完毕！！！" << endl;
}

int main(int argc, char** argv)
{
	// 加载点云文件
	PointCloud::Ptr source(new PointCloud);    // 源点云，待配准
	pcl::io::loadPCDFile("/home/slishy/Code/PCD/golden.pcd", *source);
	PointCloud::Ptr target(new PointCloud);    // 目标点云
	pcl::io::loadPCDFile("/home/slishy/Code/PCD/scene.pcd", *target);
    PointProcess pp;
    pp.SetStddevMulThresh(1);
    pp.SetK(100);
    pp.Removepoint(target);

	clock_t start = clock();
	// 随机采样固定的点云数量作为特征点
	pcl::RandomSample<PointT> rs_src;
	rs_src.setInputCloud(source);
	rs_src.setSample(5000);
	PointCloud::Ptr key_src(new PointCloud);
	rs_src.filter(*key_src);
	pcl::RandomSample<PointT> rs_tgt;
	rs_tgt.setInputCloud(target);
	rs_tgt.setSample(5000);
	PointCloud::Ptr key_tgt(new PointCloud);
	rs_tgt.filter(*key_tgt);

	//计算3dsc
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_src(new pcl::PointCloud<pcl::ShapeContext1980>());
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr sps_tgt(new pcl::PointCloud<pcl::ShapeContext1980>());
	
	computeKeyPoints3DSC(source, key_src,sps_src);
	computeKeyPoints3DSC(target, key_tgt, sps_tgt);
	cout << *sps_tgt << endl;

	//SAC配准
	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::ShapeContext1980> scia;
	scia.setInputSource(key_src);
	scia.setInputTarget(key_tgt);
	scia.setSourceFeatures(sps_src);
	scia.setTargetFeatures(sps_tgt);
	//scia.setMinSampleDistance(0.1);     // 设置样本之间的最小距离
	//scia.setNumberOfSamples(200);       // 设置每次迭代计算中使用的样本数量（可省）,可节省时间
	//scia.setCorrespondenceRandomness(6);// 在选择随机特征对应时，设置要使用的邻居的数量;
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	//pcl::io::savePCDFileASCII("sac.pcd", *sac_result);
	clock_t sac_time = clock();

	//icp配准
	PointCloud::Ptr icp_result(new PointCloud);
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(key_src);
	icp.setInputTarget(key_tgt);
	
	icp.setMaxCorrespondenceDistance(0.04);
	icp.setMaximumIterations(35);        // 最大迭代次数
	icp.setTransformationEpsilon(1e-10); // 两次变化矩阵之间的差值
	icp.setEuclideanFitnessEpsilon(0.01);// 均方误差
	icp.align(*icp_result, sac_trans);
	clock_t end = clock();
	cout << "total time: " << (double)(end - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "sac time: " << (double)(sac_time - start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "icp time: " << (double)(end - sac_time) / (double)CLOCKS_PER_SEC << " s" << endl;

	cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	cout << icp_trans << endl;
	//使用创建的变换对未过滤的输入点云进行变换
	pcl::transformPointCloud(*source, *icp_result, icp_trans);
	//保存转换的输入点云
	//pcl::io::savePCDFileASCII("transformed_sac.pcd", *icp_result);

	//可视化
	visualize_pcd(source, target, icp_result);

	return (0);
}


