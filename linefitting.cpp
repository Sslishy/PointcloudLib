
/*#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/voxel_grid.h>
#include <queue>

using namespace pcl;
using namespace Eigen;
using namespace std;
typedef PointXYZ PointT;
class HOUGH_LINE
{
public:
	HOUGH_LINE();
	~HOUGH_LINE();
	template<typename T> void vector_sort(std::vector<T> vector_input, std::vector<size_t> &idx);
	inline void setinputpoint(PointCloud<PointT>::Ptr point_);
	void VoxelGrid_(float size_, PointCloud<PointT>::Ptr &voxel_cloud);
	void draw_hough_spacing();
	void HOUGH_line(int x_setp_num, double y_resolution, int grid_point_number_threshold, vector<float>&K_, vector<float>&B_, int line_num=-1);
	void draw_hough_line();
private:
	PointCloud<PointT>::Ptr cloud;
	int point_num;
	PointT point_min;
	PointT point_max;
	vector<pair<double, double>>result_;
};

template<typename T>void
HOUGH_LINE::vector_sort(std::vector<T> vector_input, std::vector<size_t> &idx){
	idx.resize(vector_input.size());
	iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(), [&vector_input](size_t i1, size_t i2) { return vector_input[i1] > vector_input[i2]; });
}
inline void HOUGH_LINE::setinputpoint(PointCloud<PointT>::Ptr point_){
	cloud = point_;
	point_num = cloud->size();
}

HOUGH_LINE::HOUGH_LINE()
{
}

HOUGH_LINE::~HOUGH_LINE()
{
	cloud->clear();
	result_.clear();
}
void HOUGH_LINE::VoxelGrid_(float size_,PointCloud<PointT>::Ptr &voxel_cloud){
	PointCloud<PointT>::Ptr tem_voxel_cloud(new PointCloud<PointT>);
	VoxelGrid<PointT> vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(size_, size_, size_);
	vox.filter(*tem_voxel_cloud);
	cloud = tem_voxel_cloud;
	voxel_cloud = tem_voxel_cloud;
	point_num = cloud->size();
}
void HOUGH_LINE::draw_hough_spacing(){
	visualization::PCLPlotter *plot_(new visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
	plot_->setBackgroundColor(1, 1, 1);
	plot_->setTitle("hough space");
	plot_->setXTitle("angle");
	plot_->setYTitle("rho");
	vector<pair<double, double>>data_;
	double x_resolution1 = M_PI / 181;
	double a_1, b_1;
	for (int i_point = 0; i_point < point_num; i_point++)
	{
		a_1 = cloud->points[i_point].x;
		b_1 = cloud->points[i_point].y;
		for (int i = 0; i < 181; i++)
		{
			data_.push_back(make_pair(i*x_resolution1, a_1 * cos(i*x_resolution1) + b_1 * sin(i*x_resolution1)));
		}
		plot_->addPlotData(data_,"line",vtkChart::LINE);//X,Y均为double型的向量
		data_.clear();
	}
	plot_->setShowLegend(false);
	plot_->plot();//绘制曲线
}
void HOUGH_LINE::HOUGH_line(int x_setp_num, double y_resolution, int grid_point_number_threshold, vector<float>&K_, vector<float>&B_,int line_num){
	vector<vector<int>>all_point_row_col;
	vector<int> Grid_Index;
	getMinMax3D(*cloud, point_min, point_max);
	double x_resolution = M_PI / x_setp_num;
	int raster_rows, raster_cols;
	raster_rows = ceil((M_PI - 0) / x_resolution);
	raster_cols = ceil((point_max.y + point_max.x) / y_resolution) * 2;
	MatrixXi all_row_col(raster_cols, raster_rows);//存储每个格网内的个数
	all_row_col.setZero();
	MatrixXd all_row_col_mean(raster_cols, raster_rows);//存储每个格网内的均值
	//统计每个格网内的数量，和rho均值
	all_row_col_mean.setZero();
	double a_, b_;
	for (int i_point = 0; i_point < point_num; i_point++){
		a_ = cloud->points[i_point].x;
		b_ = cloud->points[i_point].y;
		for (int i_ = 0; i_ < x_setp_num; i_++)
		{
			double theta = 0 + i_*x_resolution + x_resolution / 2;
			double rho = a_ * cos(theta) + b_ * sin(theta);
			//double rho = line_(theta,a_,b_);
			double idx = ceil(abs(rho / y_resolution));
			if (rho >= 0)
			{
				all_row_col(raster_cols / 2 - idx, i_) += 1;
				all_row_col_mean(raster_cols / 2 - idx, i_) += rho;
			}
			else{
				all_row_col(raster_cols / 2 + idx, i_) += 1;
				all_row_col_mean(raster_cols / 2 + idx, i_) += rho;
			}
		}
	}
	//求解邻域
	int min_num_threshold = grid_point_number_threshold;
	vector<pair<double, double>>result_jz;
	//vector<pair<double, double>>result_;
	vector<int>tem_nebor;
	vector<int>num_grid;
	for (int i_row = 0; i_row < all_row_col.rows(); i_row++)
	{
		for (int i_col = 0; i_col < all_row_col.cols(); i_col++)
		{
			if (i_row == 0)
			{
				if (i_col == 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
				}
				if (i_col == all_row_col.cols() - 1)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
				}
				if (i_col != all_row_col.cols() - 1 && i_col != 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
				}
			}
			if (i_row == all_row_col.rows() - 1)
			{
				if (i_col == 0)
				{
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
				}
				if (i_col == all_row_col.cols() - 1)
				{
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
				}
				if (i_col != all_row_col.cols() - 1 && i_col != 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
				}
			}
			if (i_row != all_row_col.rows() - 1 && i_row != 0)
			{
				if (i_col == 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
				}
				if (i_col == all_row_col.cols() - 1)
				{
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
				}
				if (i_col != all_row_col.cols() - 1 && i_col != 0)
				{
					tem_nebor.push_back(all_row_col(i_row, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row - 1, i_col + 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col - 1));
					tem_nebor.push_back(all_row_col(i_row + 1, i_col + 1));
				}
			}
			int max_tem = *max_element(tem_nebor.begin(), tem_nebor.end());
			tem_nebor.clear();
			if (all_row_col(i_row, i_col) > max_tem)
			{
				num_grid.push_back(all_row_col(i_row, i_col));
				double tem_ = all_row_col_mean(i_row, i_col) / all_row_col(i_row, i_col);
				result_jz.push_back(make_pair(0 + i_col*x_resolution + x_resolution / 2, tem_));
				if (all_row_col(i_row, i_col) > min_num_threshold)
				{
					result_.push_back(make_pair(0 + i_col*x_resolution + x_resolution / 2, tem_));
				}
			}
		}
	}
	if (line_num!=-1)
	{
		result_.clear();
		vector<size_t>idx_;
		vector_sort(num_grid, idx_);
		if (result_jz.size() < line_num)
		{
			line_num = result_jz.size();
		}
		for (int i_ = 0; i_ < line_num; i_++)
		{
			result_.push_back(result_jz[idx_[i_]]);
		}
	}
	for (int i_hough = 0; i_hough < result_.size(); i_hough++){
		B_.push_back(result_[i_hough].second / sin(result_[i_hough].first));
		K_.push_back(-cos(result_[i_hough].first) / sin(result_[i_hough].first));
	}
}
void HOUGH_LINE::draw_hough_line(){
	visualization::PCLPlotter *plot_line(new visualization::PCLPlotter);
	plot_line->setBackgroundColor(1, 1, 1);
	plot_line->setTitle("line display");
	plot_line->setXTitle("x");
	plot_line->setYTitle("y");
	vector<double>x_, y_;
	for (int i_point = 0; i_point < point_num; i_point++)
	{
		x_.push_back(cloud->points[i_point].x);
		y_.push_back(cloud->points[i_point].y);
	}
	for (int i_hough = 0; i_hough < result_.size(); i_hough++){
		std::vector<double> func1(2, 0);
		func1[0] = result_[i_hough].second / sin(result_[i_hough].first);
		func1[1] = -cos(result_[i_hough].first) / sin(result_[i_hough].first);
		plot_line->addPlotData(func1, point_min.x, point_max.x);
	}
	plot_line->addPlotData(x_, y_, "display", vtkChart::POINTS);//X,Y均为double型的向量
	plot_line->setShowLegend(false);
	plot_line->plot();//绘制曲线
	//plot_line->spin();
}
int main()
{
	PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
	pcl::io::loadPCDFile<PointT>("/home/slishy/Code/PCD/hanjie/target1.pcd", *cloud);
	for(size_t i =0;i < cloud->points.size(); i++)
   {
   	cloud->points[i].x = cloud->points[i].x * 1000;
   	cloud->points[i].y = cloud->points[i].y * 1000;
   	cloud->points[i].z = cloud->points[i].z * 1000;
   }
	PointCloud<PointT>::Ptr VOXEL;
	HOUGH_LINE hough;
	hough.setinputpoint(cloud);
	hough.VoxelGrid_(1.0, VOXEL);
	//hough.draw_hough_spacing();
	vector<float>K_,B_;
	hough.HOUGH_line(181, 0.5, 400, K_, B_);//按阈值自动检测
	hough.HOUGH_line(181, 0.5, 400, K_, B_,3);//指定只选择极大值最大的3条直线
	hough.draw_hough_line();
	return 0;
}*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    // 填入点云数据
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/slishy/Code/PCD/hanjie/target2.pcd",
                               *cloud);      //拟合相机倾斜拍照的点云
//    reader.read<pcl::PointXYZ>("/home/fuhong/kfr/kinect1_pcl/ground.pcd", *cloud);            //拟合近似垂直地面的点云
//    reader.read<pcl::PointXYZ>("/home/fuhong/kfr/kinect1_pcl/ground_final.pcd", *cloud);             //拟合分割后的点云
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.001);         //设置误差容忍范围，也就是阈值
    seg.setInputCloud(cloud);               //输入点云
    seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
	seg.setOptimizeCoefficients(false);  
	seg.segment(*inliers1, *coefficients); 
    //打印直线方程
    std::cout << "a：" << coefficients->values[0] << endl;
    std::cout << "b：" << coefficients->values[1] << endl;
    std::cout << "c：" << coefficients->values[2] << endl;
    std::cout << "d：" << coefficients->values[3] << endl;
    std::cout << "e：" << coefficients->values[4] << endl;
    std::cout << "f：" << coefficients->values[5] << endl;

    /*子集提取*/
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inliers->indices.size(); ++i) {
        c_plane->points.push_back(cloud->points.at(inliers->indices[i]));
    }
	 pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane1(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inliers1->indices.size(); ++i) {
        c_plane1->points.push_back(cloud->points.at(inliers1->indices[i]));
    }
    /*单个直线时*/
    //直线点获取
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane2(new pcl::PointCloud<pcl::PointXYZ>);  //存储直线点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象
    extract.setInputCloud(cloud);    //设置输入点云
    extract.setIndices(inliers);     //设置分割后的内点为需要提取的点集
    extract.setNegative(false);      //false提取内点, true提取外点
    extract.filter(*c_plane2);        //提取输出存储到c_plane2


    // 点云可视化
    pcl::visualization::PCLVisualizer viewer;
    //viewer.addPointCloud(cloud, "cloud");  // 加载比对点云


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_plane_color(c_plane, 255, 0,
                                                                                  0);  // 设置点云颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_plane_color1(c_plane, 0, 255,
                                                                                  0);  // 设置点云颜色
    viewer.addPointCloud(c_plane, c_plane_color, "c_plane");  // 加载凹凸点云
	viewer.addPointCloud(c_plane1, c_plane_color1, "c_plane1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                            "c_plane");  // 设置点云大小
    //pcl::io::savePCDFile("/home/fuhong/Documents/projects/realsenseSR300/data/realsense_sr300/model6.pcd", *c_plane2);
    viewer.spin();
}

