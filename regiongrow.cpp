#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

/**
 * @description:				区域生长分割算法
 * @param cloud					输入点云
 * @param clusters				分割结果
 * @param smoothness_threshold	平滑度阈值
 * @param curvature_threshold	曲率阈值
 * @param min_cluster_size		最小聚类点数	
 * @param max_cluster_size		最大聚类点数
 * @param nr_k					k邻近点数
 */
void regiongrow(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
	float smoothness_threshold, float curvature_threshold, int min_cluster_size, int max_cluster_size, int nr_k)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(nr_k);
	normal_estimator.compute(*normals);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);

	sort(cloud_normals->begin(), cloud_normals->end(),[](pcl::PointXYZINormal pt1, pcl::PointXYZINormal pt2) {return pt1.curvature < pt2.curvature; });


	std::vector<int> seed_queue;
	std::vector<bool> cluster_queue(cloud_normals->points.size(), false); 
	std::vector<bool> cluster_queue_old(cloud_normals->points.size(), false);

	pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
	kdtree.setInputCloud(cloud_normals);
	std::vector<int> pointsIdx(nr_k);          
	std::vector<float> pointsDistance(nr_k);  

	int cluster_size = 0;

	for (int i = 0; i < cloud_normals->size(); ++i)
	{
		seed_queue.push_back(i);

		if (cluster_queue[i]) continue;

		++cluster_size;

		while (!seed_queue.empty())
		{
			int seed_index = seed_queue.back();
			seed_queue.pop_back();

			kdtree.nearestKSearch(cloud_normals->points[seed_index], nr_k, pointsIdx, pointsDistance);    

			Eigen::Vector3f v1(cloud_normals->points[seed_index].normal_x, cloud_normals->points[seed_index].normal_y, cloud_normals->points[seed_index].normal_z);
			for (int j = 0; j < pointsIdx.size(); ++j)
			{
				Eigen::Vector3f	v2(cloud_normals->points[pointsIdx[j]].normal_x, cloud_normals->points[pointsIdx[j]].normal_y, cloud_normals->points[pointsIdx[j]].normal_z);
				float angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2) * 180 / M_PI;
				if (fabs(angle) <= smoothness_threshold && !cluster_queue[pointsIdx[j]])
				{
					cluster_queue[pointsIdx[j]] = true;
					if (cloud_normals->points[pointsIdx[j]].curvature <= curvature_threshold && find(seed_queue.begin(), seed_queue.end(), pointsIdx[j]) == seed_queue.end())
						seed_queue.push_back(pointsIdx[j]);
				}
			}
		}

		std::vector<int> indexs;
		for (size_t i = 0; i < cluster_queue.size(); ++i)
		{
			if (cluster_queue[i] && !cluster_queue_old[i])
				indexs.push_back(i);
		}
		cluster_queue_old = cluster_queue;
	
		if (indexs.size() >= min_cluster_size && indexs.size() <= max_cluster_size)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flag(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cloud_normals, indexs, *cloud_flag);
			clusters.push_back(cloud_flag);
			std::cout << indexs.size() << std::endl;
			pcl::io::savePCDFile("cloud_flag" + std::to_string(cluster_size) + ".pcd", *cloud_flag);
		}
	}
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	pcl::io::loadPCDFile("cow.pcd", *cloud);

	regiongrow(cloud, clusters, 30, 0.05, 50, 1000000, 30);

	system("pause");
	return EXIT_SUCCESS;
}

