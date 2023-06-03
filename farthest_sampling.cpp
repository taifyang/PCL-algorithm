#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>


/**
 * @description:			最远点采样
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param num_pts			采样点数
 */
void farthestsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, int num_pts)
{
	size_t N = cloud->size();
	assert(N >= num_pts);

	srand(time(0));
	size_t seed_index = rand() % N;
	pcl::PointXYZ p = cloud->points[seed_index];;
	cloud_filtered->push_back(p);
	cloud->erase(cloud->begin() + seed_index);

	for (size_t i = 1; i < num_pts; i++)
	{
		float max_distance = 0;
		size_t max_index = 0;
		
		for (size_t j = 0; j < cloud->size(); j++)
		{
			float distance = pcl::euclideanDistance(p, cloud->points[j]);
			if (distance > max_distance)
			{
				max_distance = distance;
				max_index = max_index;
			}
		}
		p = cloud->points[max_index];
		cloud_filtered->push_back(p);
		cloud->erase(cloud->begin() + max_index);
	}
}


int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("bed_0610.pcd", *cloud);

	farthestsample(cloud, cloud_filtered, 2048);
	pcl::io::savePCDFile("myfarthestsample.pcd", *cloud_filtered);

	return EXIT_SUCCESS;
}