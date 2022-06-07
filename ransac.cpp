#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 * @description:	RANSAC算法拟合平面
 * @param cloud		输入点云
 * @param max_iter	最大迭代次数
 * @param threshold	距离阈值
 */
void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_iter, float threshold)
{
	srand(time(0)); 
	int size_old = 3;
	double a_final, b_final, c_final, d_final; 

	while (--max_iter) 
	{
		std::vector<int> index;
		for (int k = 0; k < 3; ++k)
		{
			index.push_back(rand() % cloud->size());
		}		
		auto idx = index.begin();
		double x1 = cloud->points[*idx].x, y1 = cloud->points[*idx].y, z1 = cloud->points[*idx].z; 
		++idx;
		double x2 = cloud->points[*idx].x, y2 = cloud->points[*idx].y, z2 = cloud->points[*idx].z;
		++idx;
		double x3 = cloud->points[*idx].x, y3 = cloud->points[*idx].y, z3 = cloud->points[*idx].z;

		double a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		double b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		double c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		double d = -(a*x1 + b*y1 + c*z1);

		for (auto iter = cloud->begin(); iter != cloud->end(); ++iter)
		{ 
			double dis = fabs(a*iter->x + b*iter->y + c*iter->z + d) / sqrt(a*a + b*b + c*c);
			if (dis < threshold)	index.push_back(iter - cloud->begin());
		}

		if (index.size() > size_old)
		{
			size_old = index.size();
			a_final = a; b_final = b; c_final = c; d_final = d;
		}
	}
    std::cout << a_final << " " << b_final << " " << c_final << " " << d_final << std::endl;
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("pingfeng_skeleton.pcd", *cloud);

	ransac(cloud, 50, 3);

	system("pause");
	return EXIT_SUCCESS;
}
