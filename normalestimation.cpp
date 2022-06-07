#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

/**
 * @description:	最小二乘法拟合平面
 * @param cloud		输入点云
 * @return			平面方程系数
 */
std::vector<double> FitPlaneByLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	if (cloud->points.size() < 3)	return {};

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

	return{ a / sqrt(a*a + b*b + c*c), b / sqrt(a*a + b*b + c*c) , c / sqrt(a*a + b*b + c*c), std::min(v1,std::min(v2,v3)) / (v1 + v2 + v3) };
}

/**
 * @description:	最小二乘法拟合二次曲面
 * @param cloud		输入点云
 * @param point		给定点
 * @return			二次曲面给定点初的高斯曲率
 */
double FitQuadricByLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& point)
{
	if (cloud->points.size() < 6)	return{};

	double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;	
	double u = 0, v = 0;								
	double E = 0, G = 0, F = 0, L = 0, M = 0, N = 0;	

	double mean_curvature = 0, guass_curvature = 0;		

	Eigen::Matrix<double, 6, 6> Q; 
	Eigen::Matrix<double, 6, 1> B; 

	Q.setZero();
	B.setZero();

	for (int i = 0; i < cloud->points.size(); i++)
	{
		Q(0, 0) += pow(cloud->points[i].x, 4);
		Q(1, 0) = Q(0, 1) += pow(cloud->points[i].x, 3)*cloud->points[i].y;
		Q(2, 0) = Q(0, 2) += pow(cloud->points[i].x*cloud->points[i].y, 2);
		Q(3, 0) = Q(0, 3) += pow(cloud->points[i].x, 3);
		Q(4, 0) = Q(0, 4) += pow(cloud->points[i].x, 2)*cloud->points[i].y;
		Q(5, 0) = Q(0, 5) += pow(cloud->points[i].x, 2);
		Q(1, 1) += pow(cloud->points[i].x*cloud->points[i].y, 2);
		Q(2, 1) = Q(1, 2) += pow(cloud->points[i].y, 3)*cloud->points[i].x;
		Q(3, 1) = Q(1, 3) += pow(cloud->points[i].x, 2)*cloud->points[i].y;
		Q(4, 1) = Q(1, 4) += pow(cloud->points[i].y, 2)*cloud->points[i].x;
		Q(5, 1) = Q(1, 5) += cloud->points[i].x*cloud->points[i].y;
		Q(2, 2) += pow(cloud->points[i].y, 4);
		Q(3, 2) = Q(2, 3) += pow(cloud->points[i].y, 2)*cloud->points[i].x;
		Q(4, 2) = Q(2, 4) += pow(cloud->points[i].y, 3);
		Q(5, 2) = Q(2, 5) += pow(cloud->points[i].y, 2);
		Q(3, 3) += pow(cloud->points[i].x, 2);
		Q(4, 3) = Q(3, 4) += cloud->points[i].x*cloud->points[i].y;
		Q(5, 3) = Q(3, 5) += cloud->points[i].x;
		Q(4, 4) += pow(cloud->points[i].y, 2);
		Q(5, 4) = Q(4, 5) += cloud->points[i].y;
		Q(5, 5) += 1;

		B(0, 0) += pow(cloud->points[i].x, 2)*cloud->points[i].z;
		B(1, 0) += cloud->points[i].x*cloud->points[i].y*cloud->points[i].z;
		B(2, 0) += pow(cloud->points[i].y, 2)*cloud->points[i].z;
		B(3, 0) += cloud->points[i].x*cloud->points[i].z;
		B(4, 0) += cloud->points[i].y*cloud->points[i].z;
		B(5, 0) += cloud->points[i].z;

		Eigen::Matrix<double, 6, 6> Q_inverse = Q.inverse();
		
		for (size_t j = 0; j < 6; ++j)
		{
			a += Q_inverse(0, j)*B(j, 0);
			b += Q_inverse(1, j)*B(j, 0);
			c += Q_inverse(2, j)*B(j, 0);
			d += Q_inverse(3, j)*B(j, 0);
			e += Q_inverse(4, j)*B(j, 0);
			f += Q_inverse(5, j)*B(j, 0);
		}		
	}

	u = 2 * a * point.x + b * point.y + d;
	v = 2 * c * point.y + b * point.x + e;

	E = 1 + u * u;
	F = u * v;
	G = 1 + v * v;

	L = 2 * a / sqrt(1 + u * u + v * v);
	M = b / sqrt(1 + u * u + v * v);
	N = 2 * c / sqrt(1 + u * u + v * v);

	guass_curvature = (L * N - M * M) / (E * G - F * F);

	mean_curvature = (E * N - 2 * F * M + G * L) / (2 * E * G - 2 * F * F);

	return guass_curvature;
}

/**
 * @description:	法线估计
 * @param cloud		输入点云
 * @param normals	法向量和曲率
 * @param nr_k		k邻近点数
 */
void normalestimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud <pcl::Normal>::Ptr normals, int nr_k)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointsIdx(nr_k);          
	std::vector<float> pointsDistance(nr_k);   

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		kdtree.nearestKSearch(cloud->points[i], nr_k, pointsIdx, pointsDistance);    
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud, pointsIdx, *cloud_temp);
	
		std::vector<double> norm = FitPlaneByLeastSquares(cloud_temp);	
		//double curvature = FitQuadricByLeastSquares(cloud_temp, cloud->points[i]);		

		pcl::Normal p(norm[0], norm[1], norm[2]);
		p.curvature = norm[3];
		normals->push_back(p);
	}
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	normalestimation(cloud, normals, 10);
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
	pcl::io::savePCDFile("normals.pcd", *cloud_normals);

	system("pause");
	return EXIT_SUCCESS;
}

