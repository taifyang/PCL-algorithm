#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> 
#include <pcl/filters/extract_indices.h>

/**
 * @description:	计算向量元素的均值
 * @param v			输入向量
 * @return			向量元素的均值
 */
float distavg(std::vector<float>& v)
{
	float sum = 0.0;
	for (int i = 0; i < v.size(); ++i)
	{
		sum += sqrt(v[i]);
	}
	return sum / v.size();
}

/**
 * @description:	计算向量元素的标准差
 * @param v			输入向量
 * @param avg		向量元素的均值
 * @return			向量元素的标准差
 */
float calcsigma(std::vector<float>& v, float& avg)
{
	float sigma = 0.0;
	for (int i = 0; i < v.size(); ++i)
	{
		sigma += pow(v[i] - avg, 2);
	}
	return sqrt(sigma / v.size());
}

/**
 * @description:			统计学滤波
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param nr_k				k邻近点数
 * @param std_mul			标准差乘数
 * @param negative			设置移除或者保留
 */
void statisticalremoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, 
	int nr_k, double std_mul, bool negative = false)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
    std::vector<float> avg;
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		std::vector<int> id(nr_k);
		std::vector<float> dist(nr_k);
		tree.nearestKSearch(cloud->points[i], nr_k, id, dist);
		avg.push_back(distavg(dist));
	}

	float u = accumulate(avg.begin(), avg.end(), 0.0) / avg.size(); 
	float sigma = calcsigma(avg, u);
    std::vector<int> index;
	for (int i = 0; i < cloud->points.size(); ++i)
	{
        std::vector<int> id(nr_k);
        std::vector<float> dist(nr_k);
		tree.nearestKSearch(cloud->points[i], nr_k, id, dist);
		float temp_avg = distavg(dist);
		if (temp_avg >= u - std_mul*sigma && temp_avg <= u + std_mul*sigma)
		{
			index.push_back(i);
		}
	}

	boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(index_ptr);
	extract.setNegative(negative);
	extract.filter(*cloud_filtered);
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	statisticalremoval(cloud, cloud_filtered, 50, 1.0);
	pcl::io::savePCDFile("mystatisticalremoval.pcd", *cloud_filtered);

	system("pause");
	return EXIT_SUCCESS;
}

