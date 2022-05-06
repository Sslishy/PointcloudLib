#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;
class PointCloudAligment
{
private:
    Eigen::Matrix4f m_transformation;
	Eigen::Matrix4f m_icp_transformation;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_cloud_out;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudAligment(/* args */);
    ~PointCloudAligment();
    void Aligment(pcl::PointCloud<pcl::PointXYZ>::Ptr goldsimple,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr AligmentPointCloud_);
     void Gettransformation(Eigen::Matrix4f &transformation)
    {
        transformation = this->m_transformation;
    }
};
PointCloudAligment::PointCloudAligment(/* args */)
{
    
}

PointCloudAligment::~PointCloudAligment()
{

}
void PointCloudAligment::Aligment(pcl::PointCloud<pcl::PointXYZ>::Ptr goldsimple,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr AligmentPointCloud_)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr goldsimple_(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in_(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*goldsimple,*goldsimple_);
    pcl::copyPointCloud(*cloud_in,*cloud_in_);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr goldsimple_features(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudin_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	nest.setNumberOfThreads(8);
	nest.setRadiusSearch(0.01);
	nest.setSearchMethod(tree);
	nest.setInputCloud(goldsimple_);
	nest.compute(*goldsimple_);
	nest.setInputCloud(cloud_in_);
	nest.compute(*cloud_in_);
    pcl::FPFHEstimationOMP<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(8);
	fest.setRadiusSearch(0.055);
	fest.setInputCloud(goldsimple_);
	fest.setSearchMethod(tree);
	fest.setInputNormals(goldsimple_);
	fest.compute(*goldsimple_features);
	//pcl::io::savePCDFile("goldsimple_features.pcd",*goldsimple_features);
	//pcl::io::loadPCDFile("goldsimple_features.pcd",*goldsimple_features);
	fest.setInputCloud(cloud_in_);
	fest.setInputNormals(cloud_in_);
	fest.setSearchMethod(tree);
	fest.compute(*cloudin_features);
	pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> align_;
	align_.setInputSource(goldsimple_);
	align_.setSourceFeatures(goldsimple_features);
	align_.setInputTarget(cloud_in_);
	align_.setTargetFeatures(cloudin_features);
	align_.setMaximumIterations(10000); // Number of RANSAC iterations
	align_.setNumberOfSamples(3); // 在对象和场景之间进行采样的点对应数，至少需要N个点才能进行计算
	align_.setCorrespondenceRandomness(5); // 在N个最佳匹配之间进行性随机选择
	align_.setSimilarityThreshold(0.8f); // 根据采样之间的距离位置不变的几何一致性，尽早消除不良影响
	align_.setMaxCorrespondenceDistance(2.5f * 0.005f); // 欧几里德距离阈值，用于确定变换后的点云是否正确对齐
	align_.setInlierFraction(0.85f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align_.align(*cloud_out);
        pcl::copyPointCloud(*cloud_out,*AligmentPointCloud_);
	}
	if (align_.hasConverged())
	{	
		m_transformation = align_.getFinalTransformation();
		/*pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());*/
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
	}
}