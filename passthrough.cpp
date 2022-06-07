#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

/**
 * @description:			直通滤波
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param field_name		字段名
 * @param limit_min			最小值
 * @param limit_max			最大值
 * @param limit_negative	设置移除或者保留
 */
void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered,
	std::string field_name, float limit_min, float limit_max, bool limit_negative = false)
{
	std::vector<int> index;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (field_name == "x")
		{
			if (cloud->points[i].x >= limit_min && cloud->points[i].x <= limit_max)
				index.push_back(i);
		}
		if (field_name == "y")
		{
			if (cloud->points[i].y >= limit_min && cloud->points[i].y <= limit_max)
				index.push_back(i);
		}
		if (field_name == "z")
		{
			if (cloud->points[i].z >= limit_min && cloud->points[i].z <= limit_max)
				index.push_back(i);
		}
	}

	boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(index_ptr);
	extract.setNegative(limit_negative);
	extract.filter(*cloud_filtered);
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	passthrough(cloud, cloud_filtered, "z", 0.0, 1.0);
	pcl::io::savePCDFile("mypassthrough.pcd", *cloud_filtered);

	system("pause");
	return EXIT_SUCCESS;
}

