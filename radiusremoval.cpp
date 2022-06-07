#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

/**
 * @description:			半径滤波
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param radius			给定半径
 * @param min_pts			最小点数
 */
void radiusremoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, double radius, int min_pts)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	for (int i = 0; i < cloud->points.size(); ++i)
	{
        std::vector<int> id(min_pts);
        std::vector<float> dist(min_pts);
		tree.nearestKSearch(cloud->points[i], min_pts, id, dist);
		if (dist[dist.size() - 1] < radius)
		{
			cloud_filtered->push_back(cloud->points[i]);
		}
	}
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	radiusremoval(cloud, cloud_filtered, 1, 200);
	pcl::io::savePCDFile("myradiusremoval.pcd", *cloud_filtered);

	system("pause");
	return EXIT_SUCCESS;
}

