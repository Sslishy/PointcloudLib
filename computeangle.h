#pragma once
#include<iostream>
#include<string>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include<pcl/PCLPointCloud2.h>
#include <Eigen/Dense>
using namespace std;
using namespace cv;
class computeangle
{
public:
    inline const void SetRotate(float Angle)
    {
        m_Angle = Angle;
    }
    inline const void Setoffset_x_value(float offset_x_value)
    {
        m_offset_x_value = offset_x_value;
    }
    inline const void Setoffset_y_value(float offset_y_value)
    {
        m_offset_y_value = offset_y_value;
    }
    inline const void Setoffset_z_value(float offset_z_value)
    {
        m_offset_z_value = offset_z_value;
    }
    computeangle();
    void ComputeDirection(const vector<pcl::Normal> Normal_Z, vector<pcl::Normal>& Normal_X, vector<pcl::Normal>& Normal_Y);
    void computeRPY(const vector<pcl::Normal> Normal_Z, const vector<pcl::Normal> Normal_X, const vector<pcl::Normal> Normal_Y, vector<cv::Vec3f>& RPYList);
    void offsetX(vector<pcl::PointXYZ> &center,const vector<pcl::Normal> Normal_X);
    void offsetY(vector<pcl::PointXYZ> &center,const vector<pcl::Normal> Normal_Y);
    void offsetZ(vector<pcl::PointXYZ> &center,const vector<pcl::Normal> Normal_Z);
    Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix4f& R);
    Eigen::Matrix4f eulerAnglesToRotationMatrix(const Eigen::Vector3f theta_);
    cv::Mat eulerAnglesToRotationMatrix(const cv::Vec3f theta_);
    cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);
    Eigen::Quaterniond EulerAngles2Quaterniond(cv::Vec3f theta);
    ~computeangle();
private:
    float m_offset_x_value;
    float m_offset_y_value;
    float m_offset_z_value;
    float m_Angle;
    vector<pcl::Normal> m_Normal_Z;
    vector<pcl::Normal> m_Normal_Y;
    vector<pcl::Normal> m_Normal_X;
};
Eigen::Quaterniond computeangle::EulerAngles2Quaterniond(cv::Vec3f theta)
{
    Vec3f theta_ = theta;
	theta_ /= 180 / CV_PI;
    Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd(theta_[2], Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(theta_[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(theta_[0], Eigen::Vector3d::UnitX());
    return quaternion3;
}
