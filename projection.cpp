#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

/**
 * @description:			点云投影
 * @param cloud				输入点云
 * @param cloud_projected	投影点云
 * @param coefficients		投影平面方程系数
 */
void projection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_projected, pcl::ModelCoefficients::Ptr coefficients)
{
	float a = coefficients->values[0], b = coefficients->values[1], c = coefficients->values[2], d = coefficients->values[3];
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		float x0 = cloud->points[i].x;
		float y0 = cloud->points[i].y;
		float z0 = cloud->points[i].z;
		float xp = ((b*b + c*c)*x0 - a*(b*y0 + c*z0 + d)) / (a*a + b*b + c*c);
		float yp = ((a*a + c*c)*y0 - b*(a*x0 + c*z0 + d)) / (a*a + b*b + c*c);
		float zp = ((a*a + b*b)*z0 - c*(a*x0 + b*y0 + d)) / (a*a + b*b + c*c);
		cloud_projected->push_back(pcl::PointXYZ(xp, yp, zp));
	}
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = coefficients->values[3];
	coefficients->values[2] = 1.0;
	projection(cloud, cloud_filtered, coefficients);
	pcl::io::savePCDFile("myprojection.pcd", *cloud_filtered);

	system("pause");
	return EXIT_SUCCESS;
}

