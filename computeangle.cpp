#include "computeangle.h"
computeangle::computeangle()
{
	m_Angle = 0;
	m_offset_x_value = 0;
	m_offset_y_value = 0;
	m_offset_z_value = 0;

}
void computeangle::offsetX(vector<pcl::PointXYZ> &center,const vector<pcl::Normal> Normal_X)
{
	pcl::PointXYZ offpoint;
	float x = m_offset_x_value * 0.001;
	for (size_t i = 0; i < Normal_X.size(); i++)
	{
		offpoint.x = x * Normal_X[i].normal_x;
		offpoint.y = x * Normal_X[i].normal_y;
		offpoint.z = x * Normal_X[i].normal_z;
		center[i].x = center[i].x + offpoint.x;
		center[i].y = center[i].y + offpoint.y;
		center[i].z = center[i].z + offpoint.z;
	}
}
void computeangle::offsetY(vector<pcl::PointXYZ> &center,const vector<pcl::Normal> Normal_Y)
{
	pcl::PointXYZ offpoint;
	float x = m_offset_y_value * 0.001;
	for (size_t i = 0; i < Normal_Y.size(); i++)
	{
		offpoint.x = x * Normal_Y[i].normal_x;
		offpoint.y = x * Normal_Y[i].normal_y;
		offpoint.z = x * Normal_Y[i].normal_z;
		center[i].x = center[i].x + offpoint.x;
		center[i].y = center[i].y + offpoint.y;
		center[i].z = center[i].z + offpoint.z;
	}
}
void computeangle::offsetZ(vector<pcl::PointXYZ> &center,const vector<pcl::Normal> Normal_Z)
{
	pcl::PointXYZ offpoint;
	float off = m_offset_z_value * 0.001;
	float x_o,y_o,z_o,x1,y1,z1;
	for (size_t i = 0; i < Normal_Z.size(); i++)
	{
		x_o = Normal_Z[i].normal_x + center[0].x;
		y_o = Normal_Z[i].normal_y + center[0].y;
		z_o = Normal_Z[i].normal_z + center[0].z;
		x1 = off * x_o - off * center[0].x + center[0].x;
		y1 = off * y_o - off * center[0].y + center[0].y;
		z1 = off * z_o - off * center[0].z + center[0].z;
		center[i].x = x1;
		center[i].y = y1;
		center[i].z = z1;
	}
	
}
Eigen::Vector3f computeangle::rotationMatrixToEulerAngles(Eigen::Matrix3f& R)
{
	//assert(isRotationMatrix(R));
	float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
		x = (x * 180) / M_PI;
		y = (y * 180) / M_PI;
		z = (z * 180) / M_PI;
	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
		x = (x * 180) / M_PI;
		y = (y * 180) / M_PI;
		z = (z * 180) / M_PI;
	}
	return Eigen::Vector3f(x, y, z);
}
cv::Vec3f computeangle::rotationMatrixToEulerAngles(cv::Mat& R)
{
	//assert(isRotationMatrix(R));
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
		x = (x * 180) / CV_PI;
		y = (y * 180) / CV_PI;
		z = (z * 180) / CV_PI;
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
		x = (x * 180) / CV_PI;
		y = (y * 180) / CV_PI;
		z = (z * 180) / CV_PI;
	}
	return cv::Vec3f(x, y, z);
}
void computeangle::ComputeDirection(const vector<pcl::Normal> Normal_Z, vector<pcl::Normal>& Normal_X, vector<pcl::Normal>& Normal_Y)
{
	Normal_X.resize(Normal_Z.size());
	Normal_Y.resize(Normal_Z.size());
	//object angle 
	double Angle2 = m_Angle;
	double Normal_X1, Normal_Y1;
	//�����ƫת���ٶ���ԭʼ����ϵ��XY�ĵ������
	Normal_Y1 = sin((CV_PI / 180) * Angle2);
	Normal_X1 = cos((CV_PI / 180) * Angle2);
	double DorectionP_X, DorectionP_Y, DorectionP_Distance, DorectionP_Distance1;
	double x;
	double Angle;
	int Angle1;
	for (size_t i = 0; i < Normal_Z.size(); i++)
	{
		//����㵽������ƽ�淽�̵ľ���
		DorectionP_Distance = (-(Normal_Z[i].normal_x * Normal_X1 + Normal_Z[i].normal_y * Normal_Y1)) / Normal_Z[i].normal_z;
		x = atan(DorectionP_Distance);
		Angle = (x / CV_PI) * 180; 
		if (Normal_X1 < 0)
		{
			DorectionP_X = -sqrt(Normal_X1 * Normal_X1 * cos(x) * cos(x));
		}
		else
		{
			DorectionP_X = sqrt(Normal_X1 * Normal_X1 * cos(x) * cos(x));
		}
		if (Normal_Y1 < 0)
		{
			DorectionP_Y = -sqrt(Normal_Y1 * Normal_Y1 * cos(x) * cos(x));
		}
		else
		{
			DorectionP_Y = sqrt(Normal_Y1 * Normal_Y1 * cos(x) * cos(x));
		}
		DorectionP_Distance1 = sin(x);
		Normal_X[i].normal_x = DorectionP_X;
		Normal_X[i].normal_y = DorectionP_Y;
		Normal_X[i].normal_z = DorectionP_Distance1;
		//�������
		Normal_Y[i].normal_x = Normal_X[i].normal_y * Normal_Z[i].normal_z - Normal_X[i].normal_z * Normal_Z[i].normal_y;
		Normal_Y[i].normal_y = Normal_X[i].normal_z * Normal_Z[i].normal_x - Normal_X[i].normal_x * Normal_Z[i].normal_z;
		Normal_Y[i].normal_z = Normal_X[i].normal_x * Normal_Z[i].normal_y - Normal_X[i].normal_y * Normal_Z[i].normal_x;
	}
}
void computeangle::ComputeRotationAngle(Eigen::Vector3f & RPY)
{
	Eigen::Matrix3f rotation_matrix;
	rotation_matrix = eulerAnglesToRotationMatrix(RPY);
	Eigen::Matrix3f rz_matrix;
	Eigen::Vector3f rzangle;
	rzangle[0] = 0;rzangle[1] = 0;
	rzangle[2] = m_Angle; 
	rz_matrix = eulerAnglesToRotationMatrix(rzangle);
	rotation_matrix = rotation_matrix *rz_matrix ;
	RPY = rotationMatrixToEulerAngles(rotation_matrix);

}
void computeangle::computeRPY(const vector<pcl::Normal> Normal_Z, const vector<pcl::Normal> Normal_X, const vector<pcl::Normal> Normal_Y, vector<cv::Vec3f>& RPYList)
{
	for (size_t i = 0; i < Normal_Z.size(); i++)
	{
		cv::Mat_<double> R = (cv::Mat_<double>(3, 3) <<

			Normal_X[i].normal_x, Normal_X[i].normal_y, Normal_X[i].normal_z,
			Normal_Y[i].normal_x, Normal_Y[i].normal_y, Normal_Y[i].normal_z,
			Normal_Z[i].normal_x, Normal_Z[i].normal_y, Normal_Z[i].normal_z
			);
		cv::Vec3f RPY;
		//����λ����ת����ŷ����
		RPY = rotationMatrixToEulerAngles(R);
		RPYList[i] = RPY;
	}
}
void computeangle::computeRPY(const pcl::Normal Normal_Z,  Eigen::Vector3f & RPY)
{
	Eigen::Matrix3f rotation_matrix;
		rotation_matrix << 
		1, 0, 0, 
		0, 1, 0,
		Normal_Z.normal_x, Normal_Z.normal_y, Normal_Z.normal_z;
		Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
		Eigen::Matrix3f R_ = R.cast<float>();
		rotation_matrix =  rotation_matrix * R_;
		RPY = rotationMatrixToEulerAngles(rotation_matrix);
		RPY[2] = 0;
}
Eigen::Matrix3f computeangle::eulerAnglesToRotationMatrix(const Eigen::Vector3f theta_)
{
	Eigen::Vector3f theta = theta_;
	theta /= 180 / M_PI;
	// Calculate rotation about x axis
	Eigen::Matrix3f R_x;
	R_x(0,0) = 1;
	R_x(0,1) = 0;
	R_x(0,2) = 0;
	R_x(1,0) = 0;
	R_x(1,1) = cos(theta[0]);
	R_x(1,2) = -sin(theta[0]);
	R_x(2,0) = 0;
	R_x(2,1) = sin(theta[0]);
	R_x(2,2) = cos(theta[0]);

	// Calculate rotation about y axis

	Eigen::Matrix3f R_y;
	R_y(0,0) = cos(theta[1]);
	R_y(0,1) = 0;
	R_y(0,2) = sin(theta[1]);
	R_y(1,0) = 0;
	R_y(1,1) = 1;
	R_y(1,2) = 0;
	R_y(2,0) = -sin(theta[1]);
	R_y(2,1) = 0;
	R_y(2,2) = cos(theta[1]);

	// Calculate rotation about z axis
	Eigen::Matrix3f R_z;
	R_z(0,0) = cos(theta[2]);
	R_z(0,1) = -sin(theta[2]);
	R_z(0,2) = 0;
	R_z(1,0) = sin(theta[2]);
	R_z(1,1) = cos(theta[2]);
	R_z(1,2) = 0;
	R_z(2,0) = 0;
	R_z(2,1) = 0;
	R_z(2,2) = 1;
	// Combined rotation matrix
	Eigen::Matrix3f R_ ;
	R_= R_z * R_y * R_x;
	return R_;
}
cv::Mat computeangle::eulerAnglesToRotationMatrix(const cv::Vec3f theta_)
{
	Vec3f theta = theta_;
	theta /= 180 / CV_PI;
	// Calculate rotation about x axis
	Mat R_x = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);

	// Combined rotation matrix
	Mat R_ = R_x * R_y * R_z;
	return R_;
}
computeangle::~computeangle()
{
	
}