#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 * @@description:	最小二乘法拟合平面
 * @param cloud		输入点云
 */
void FitPlaneByLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	if (cloud->points.size() < 3)	return;

	double meanX = 0, meanY = 0, meanZ = 0;
	double meanXX = 0, meanYY = 0, meanZZ = 0;
	double meanXY = 0, meanXZ = 0, meanYZ = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		meanX += cloud->points[i].x;
		meanY += cloud->points[i].y;
		meanZ += cloud->points[i].z;

		meanXX += cloud->points[i].x * cloud->points[i].x;
		meanYY += cloud->points[i].y * cloud->points[i].y;
		meanZZ += cloud->points[i].z * cloud->points[i].z;

		meanXY += cloud->points[i].x * cloud->points[i].y;
		meanXZ += cloud->points[i].x * cloud->points[i].z;
		meanYZ += cloud->points[i].y * cloud->points[i].z;
	}
	meanX /= cloud->points.size();
	meanY /= cloud->points.size();
	meanZ /= cloud->points.size();
	meanXX /= cloud->points.size();
	meanYY /= cloud->points.size();
	meanZZ /= cloud->points.size();
	meanXY /= cloud->points.size();
	meanXZ /= cloud->points.size();
	meanYZ /= cloud->points.size();

	Eigen::Matrix3d m;
	m(0, 0) = meanXX - meanX * meanX; m(0, 1) = meanXY - meanX * meanY; m(0, 2) = meanXZ - meanX * meanZ;
	m(1, 0) = meanXY - meanX * meanY; m(1, 1) = meanYY - meanY * meanY; m(1, 2) = meanYZ - meanY * meanZ;
	m(2, 0) = meanXZ - meanX * meanZ; m(2, 1) = meanYZ - meanY * meanZ; m(2, 2) = meanZZ - meanZ * meanZ;
	Eigen::EigenSolver<Eigen::Matrix3d> PlMat(m * cloud->points.size());
        Eigen::Matrix3d eigenvalue = PlMat.pseudoEigenvalueMatrix();
        Eigen::Matrix3d eigenvector = PlMat.pseudoEigenvectors();

	double v1 = eigenvalue(0, 0), v2 = eigenvalue(1, 1), v3 = eigenvalue(2, 2);
	int minNumber = 0;
	if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3)))	minNumber = 1;
	if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2)))	minNumber = 2;
	double a = eigenvector(0, minNumber), b = eigenvector(1, minNumber), c = eigenvector(2, minNumber), d = -(a * meanX + b * meanY + c * meanZ);

	if (c < 0)
	{
		a *= -1.0;
		b *= -1.0;
		c *= -1.0;
		d *= -1.0;
	}
    std::cout << a << " " << b << " " << c << " " << d << std::endl;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("pingfeng_skeleton.pcd", *cloud);

	FitPlaneByLeastSquares(cloud);

	system("pause");
	return EXIT_SUCCESS;
}
